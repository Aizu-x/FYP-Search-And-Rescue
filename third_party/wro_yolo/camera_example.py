"""Minimal example for robot-side integration (camera only; no motion control).

Works if the camera is accessible to OpenCV using the selected camera index.
Camera setup is owned by the teammate. Detecting an object does not determine
its 3D position or its occupancy-grid cell.
"""
import cv2
from wro_detector import WRODetector

detector = WRODetector(conf_threshold=0.25)
cap = cv2.VideoCapture(0)  # Teammate must change camera source if needed.
if not cap.isOpened():
    raise RuntimeError("Camera 0 unavailable; check MentorPi camera source")

try:
    for frame_number in range(30):  # Stop automatically after 30 frames.
        ok, frame = cap.read()
        if not ok:
            break
        detections = detector.predict(frame)
        # Supply to your own code; do NOT interpret bbox as meters or A* grid cells.
        print({"frame_number": frame_number, "detections": detections})
finally:
    cap.release()
