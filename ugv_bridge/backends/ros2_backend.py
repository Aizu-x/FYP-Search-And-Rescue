"""ROS 2 backend: forwards velocity to HiWonder's existing controller topic and reads sensors.

UNTESTED ON THE REAL MENTORPI. Topic names come from the bridge config and must
be confirmed first with ``tools/discover_mentorpi.sh``. This node only
publishes on the configured velocity topic; it never starts, stops or
reconfigures HiWonder nodes, drivers, firmware or services.

Subscriptions use best-effort QoS, which is compatible with both reliable and
best-effort publishers. The velocity publisher is reliable with depth 1, which
is compatible with reliable and best-effort subscribers.
"""
from __future__ import annotations

import math
import threading
import time

from .base import Backend, SensorSnapshot


# MentorPi: the raw topic is colour (yuv422_yuy2); HiWonder's image_compressed is BLACK-AND-WHITE
# ('jpeg compressed mono8'), so the bridge JPEG-encodes the raw colour frames itself.
DEFAULT_CAMERA_TOPIC = '/ascamera/camera_publisher/rgb0/image'
DEFAULT_CAMERA_INFO_TOPIC = '/ascamera/camera_publisher/rgb0/camera_info'


def encode_raw_image(data, encoding, width, height, step, quality=75):
    """Convert a raw ROS image (yuv422_yuy2 / uyvy / rgb8 / bgr8 / mono8) to colour JPEG bytes (needs OpenCV)."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None
    enc = (encoding or '').lower()
    rows = np.frombuffer(data, dtype=np.uint8)[:height * step].reshape(height, step)
    if enc in ('yuv422_yuy2', 'yuyv', 'yuy2'):
        bgr = cv2.cvtColor(rows[:, :width * 2].reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2)
    elif enc in ('yuv422', 'uyvy'):
        bgr = cv2.cvtColor(rows[:, :width * 2].reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY)
    elif enc == 'rgb8':
        bgr = cv2.cvtColor(rows[:, :width * 3].reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    elif enc == 'bgr8':
        bgr = rows[:, :width * 3].reshape(height, width, 3)
    elif enc == 'mono8':
        bgr = rows[:, :width]
    else:
        return None
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Ros2Backend(Backend):
    name = 'ros2'

    def __init__(self, topics: dict, node_name='edge_nav_bridge'):
        self.topics = topics
        self.node_name = node_name
        self._lock = threading.Lock()
        self._snapshot = SensorSnapshot()
        self._thread = None
        self._image = None        # (bytes, format, monotonic time)
        self._raw = None          # newest raw frame (data, encoding, width, height, step, time)
        self._camera_info = None

    def start(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan

        self._Twist = Twist
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node(self.node_name)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._pub = self.node.create_publisher(Twist, self.topics['cmd_vel'], cmd_qos)
        if self.topics.get('odom'):
            self.node.create_subscription(Odometry, self.topics['odom'], self._on_odom, sensor_qos)
        if self.topics.get('imu'):
            self.node.create_subscription(Imu, self.topics['imu'], self._on_imu, sensor_qos)
        if self.topics.get('scan'):
            self.node.create_subscription(LaserScan, self.topics['scan'], self._on_scan, sensor_qos)
        self._setup_camera(sensor_qos)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, name='ros2-spin', daemon=True)
        self._thread.start()

    def shutdown(self):
        try:
            self.send_velocity(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            self._executor.shutdown()
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass

    def _setup_camera(self, qos):
        """Camera is optional: CompressedImage is passed through; raw Image needs OpenCV to JPEG-encode."""
        topic = self.topics.get('camera', DEFAULT_CAMERA_TOPIC)
        info_topic = self.topics.get('camera_info', DEFAULT_CAMERA_INFO_TOPIC)
        if not topic:
            return
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image
        types = dict(self.node.get_topic_names_and_types())
        default_kind = 'sensor_msgs/msg/CompressedImage' if 'compressed' in topic else 'sensor_msgs/msg/Image'
        kind = types.get(topic, [default_kind])[0]
        if kind.endswith('CompressedImage'):
            self.node.create_subscription(CompressedImage, topic, self._on_compressed, qos)
        elif kind.endswith('/Image'):
            self.node.create_subscription(Image, topic, self._on_raw_image, qos)
        else:
            print('camera topic %s has unsupported type %s; camera disabled' % (topic, kind))
            return
        if info_topic:
            self.node.create_subscription(CameraInfo, info_topic, self._on_camera_info, qos)
        print('camera: %s (%s)' % (topic, kind))

    def _on_compressed(self, msg):
        with self._lock:
            self._image = (bytes(msg.data), msg.format or 'jpeg', time.monotonic())

    def _on_raw_image(self, msg):
        # Cheap: keep only the newest raw frame. Encoding happens in latest_image(), i.e. only for the
        # frames the edge actually requests (about 8 per second), not for all 25 camera frames.
        with self._lock:
            self._raw = (bytes(msg.data), msg.encoding, msg.width, msg.height, msg.step, time.monotonic())

    def _on_camera_info(self, msg):
        with self._lock:
            self._camera_info = dict(width=int(msg.width), height=int(msg.height), k=[float(v) for v in msg.k],
                                     d=[float(v) for v in msg.d], frame_id=msg.header.frame_id)

    def latest_image(self):
        with self._lock:
            raw, image = getattr(self, '_raw', None), self._image
        if raw is not None and (image is None or image[2] != raw[5]):
            data, encoding, width, height, step, stamp = raw
            jpeg = encode_raw_image(data, encoding, width, height, step)
            if jpeg is not None:
                image = (jpeg, 'jpeg', stamp)
                with self._lock:
                    self._image = image
        return image

    def camera_info(self):
        with self._lock:
            return self._camera_info

    # ROS callbacks: store values with this process's monotonic receive time.
    def _on_odom(self, msg):
        p, t = msg.pose.pose, msg.twist.twist
        odom = dict(x=p.position.x, y=p.position.y, yaw=_yaw_from_quaternion(p.orientation),
                    vx=t.linear.x, vy=t.linear.y, wz=t.angular.z)
        with self._lock:
            self._snapshot.odom, self._snapshot.odom_time = odom, time.monotonic()

    def _on_imu(self, msg):
        imu = dict(yaw=_yaw_from_quaternion(msg.orientation), wz=msg.angular_velocity.z)
        with self._lock:
            self._snapshot.imu, self._snapshot.imu_time = imu, time.monotonic()

    def _on_scan(self, msg):
        scan = dict(angle_min=msg.angle_min, angle_increment=msg.angle_increment, ranges=list(msg.ranges),
                    range_min=msg.range_min, range_max=msg.range_max)
        with self._lock:
            self._snapshot.scan, self._snapshot.scan_time = scan, time.monotonic()

    def send_velocity(self, vx, vy, wz):
        msg = self._Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self._pub.publish(msg)

    def latest(self):
        with self._lock:
            s = self._snapshot
            return SensorSnapshot(s.odom, s.odom_time, s.imu, s.imu_time, s.scan, s.scan_time)
