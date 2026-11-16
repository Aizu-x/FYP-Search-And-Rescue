"""Portable WRO obstacle detector: two-class YOLOv5n ONNX inference.

Input: an OpenCV frame in BGR, with any resolution.
Output: a list of image-coordinate object detections.

The ONNX output has shape (1, 25200, 7) for 640x640, with each row
[x_center, y_center, width, height, objectness, p_green, p_red].
ONNX does NOT include non-maximum suppression, so this module performs it.
"""
from __future__ import annotations

from pathlib import Path
import cv2
import numpy as np
import onnxruntime as ort

CLASS_NAMES = ("green_obstacle", "red_obstacle")


def _nms_xyxy(boxes, scores, iou_threshold, max_detections=300):
    """Return indices into boxes after greedy NMS, highest confidence first."""
    if len(boxes) == 0:
        return np.empty((0,), dtype=np.int64)
    areas = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])
    order = np.argsort(-scores)
    keep = []
    while order.size and len(keep) < max_detections:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        other = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[other, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[other, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[other, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[other, 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[other] - inter + 1e-9)
        order = other[iou <= iou_threshold]
    return np.asarray(keep, dtype=np.int64)


class WRODetector:
    def __init__(self, model_path=None, conf_threshold=0.25, iou_threshold=0.45):
        if model_path is None:
            model_path = Path(__file__).resolve().parent / "models" / "best.onnx"
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"ONNX model missing: {model_path}")
        if not 0.0 < conf_threshold < 1.0:
            raise ValueError("conf_threshold must be between 0 and 1")
        if not 0.0 < iou_threshold < 1.0:
            raise ValueError("iou_threshold must be between 0 and 1")
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        if len(input_shape) != 4 or input_shape[-2:] != [640, 640]:
            raise ValueError(f"This package expects exported 640x640 ONNX input, got {input_shape}")

    def predict(self, frame_bgr):
        """Detect obstacles; returns dictionaries using ORIGINAL frame pixels.

        Detection fields: class_id, class_name, confidence, bbox_xyxy,
        bbox_xywh, center_xy.  Bounding boxes are NOT real-world positions.
        Empty list means no accepted detections, not guaranteed free space.
        """
        if frame_bgr is None or not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("Expected nonempty HxWx3 BGR numpy array")
        height, width = frame_bgr.shape[:2]
        if height < 1 or width < 1:
            raise ValueError("Empty input frame")
        size = 640
        ratio = min(size / height, size / width)
        new_w, new_h = int(round(width * ratio)), int(round(height * ratio))
        dw, dh = (size - new_w) / 2, (size - new_h) / 2
        resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        x = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        output = self.session.run([self.output_name], {self.input_name: x})[0]
        if output.ndim != 3 or output.shape[0] != 1 or output.shape[2] != 7:
            raise RuntimeError(f"Unexpected ONNX prediction shape: {output.shape}")
        pred = output[0]
        class_ids = np.argmax(pred[:, 5:7], axis=1)
        confidences = pred[:, 4] * pred[np.arange(pred.shape[0]), 5 + class_ids]
        valid = (confidences >= self.conf_threshold) & np.isfinite(confidences) & np.isfinite(pred[:, :4]).all(axis=1)
        pred, class_ids, confidences = pred[valid], class_ids[valid], confidences[valid]
        if not len(pred):
            return []
        # Limit candidates for predictable CPU performance before NMS.
        if len(pred) > 3000:
            chosen = np.argsort(-confidences)[:3000]
            pred, class_ids, confidences = pred[chosen], class_ids[chosen], confidences[chosen]
        xywh = pred[:, :4]
        boxes = np.column_stack((
            xywh[:, 0] - xywh[:, 2] / 2,
            xywh[:, 1] - xywh[:, 3] / 2,
            xywh[:, 0] + xywh[:, 2] / 2,
            xywh[:, 1] + xywh[:, 3] / 2,
        ))
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / ratio
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / ratio
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, width)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, height)
        # Greedy NMS separately for each class: don't suppress green with red.
        kept = []
        for cls in (0, 1):
            where = np.where(class_ids == cls)[0]
            kept.extend(where[_nms_xyxy(boxes[where], confidences[where], self.iou_threshold)].tolist())
        kept.sort(key=lambda idx: float(confidences[idx]), reverse=True)
        detections = []
        for i in kept[:300]:
            x1, y1, x2, y2 = [int(round(v)) for v in boxes[i]]
            if x2 <= x1 or y2 <= y1:
                continue
            cls = int(class_ids[i])
            detections.append({
                "class_id": cls,
                "class_name": CLASS_NAMES[cls],
                "confidence": round(float(confidences[i]), 6),
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_xywh": [x1, y1, x2 - x1, y2 - y1],
                "center_xy": [round((x1+x2)/2, 2), round((y1+y2)/2, 2)],
            })
        return detections


def draw_detections(frame_bgr, detections):
    """Return copy of frame with detection boxes; no robot motion commands."""
    image = frame_bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy"]
        color = (0, 180, 0) if det["class_id"] == 0 else (0, 0, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f'{det["class_name"]} {det["confidence"]:.2f}',
                    (x1, max(18, y1-7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color, 2, cv2.LINE_AA)
    return image
