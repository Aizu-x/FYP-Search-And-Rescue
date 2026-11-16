# FYP2 — WRO Red/Green Obstacle Detection Handoff

## Scope and responsibilities

This package contains the **YOLOv5n two-class object-detection component only**.
It is designed to hand off to the teammate responsible for the HiWonder
MentorPi M1 UGV, edge-device communications, mapping and A* path planning.

Class IDs:
- `0`: `green_obstacle`
- `1`: `red_obstacle`

The camera frame is supplied by the receiving application. The detector returns
**image-space coordinates**; these are **NOT robot positions, map coordinates,
metres, or occupancy-grid cells**. The teammate must perform camera calibration,
localization / projection, frame transformation and safety checks before
passing obstacle information to A* or commanding the UGV. An empty detection
list **does not prove the path is clear**. Robot safety requires independent
obstacle avoidance / sensor checks.

## Contents

- `models/best.onnx` — exported YOLOv5n with fixed input `[1,3,640,640]`
  and raw output `[1,25200,7]`, without built-in NMS.
- `models/best.pt` — PyTorch checkpoint for YOLOv5-compatible environments.
- `wro_detector.py` — self-contained ONNX Runtime inference and NMS; requires no
  YOLOv5 GitHub checkout at inference time.
- `run_detection.py` — run on a photo or a directory of photos; writes JSON and
  annotated images.
- `camera_example.py` — minimal frame integration example, no robot commands.
- `requirements.txt` — Python dependencies for ONNX CPU inference.
- `metrics.json` — logged evaluation numbers and important limitations.
- `examples/` — optional, annotated demonstration photographs.

## Setup — test first on a desktop/laptop

Use Python 3.10+ and a virtual environment. For example:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

ONNX Runtime availability on Raspberry Pi 5 depends on its Linux architecture,
Python version and installed packages. If `pip install onnxruntime` fails on the
robot, select a supported build/runtime with the robot owner; this package's
CPU test on the ASUS laptop is **not a Raspberry Pi deployment benchmark**.

## Run on images

```bash
python run_detection.py --source /path/to/photo.jpg --output outputs
python run_detection.py --source /path/to/photos --conf 0.25 --output outputs
```

Outputs: annotated images, one `.json` per input image, plus
`outputs/all_detections.json`.

## Integrate with your own Python camera loop

```python
import cv2
from wro_detector import WRODetector

detector = WRODetector("models/best.onnx", conf_threshold=0.25)
frame_bgr = cv2.imread("some_camera_frame.jpg")  # or cap.read(), etc.
detections = detector.predict(frame_bgr)
for d in detections:
    print(d["class_name"], d["confidence"], d["bbox_xyxy"])
```

Each detection resembles:

```json
{
  "class_id": 0,
  "class_name": "green_obstacle",
  "confidence": 0.94,
  "bbox_xyxy": [237, 192, 536, 853],
  "bbox_xywh": [237, 192, 299, 661],
  "center_xy": [386.5, 522.5]
}
```

The example numeric coordinates are illustrative; real values depend on the
input frame's resolution. `bbox_xyxy` is `[x_min, y_min, x_max, y_max]`, with
origin at the **top left** of the original image. For video, run `predict` once
per BGR frame. Defaults: confidence `0.25`, class-wise NMS IoU `0.45`.

## ONNX model details

- Architecture: YOLOv5n, 2 classes.
- Preprocessing: letterbox input to 640×640 with padding value 114, BGR→RGB,
  normalize values to [0,1], CHW float32, batch size 1.
- Raw ONNX prediction: `[cx, cy, w, h, objectness, p_green, p_red]`.
- Class score: `objectness × class_probability`.
- Postprocessing: confidence filter, class-wise non-maximum suppression,
  unletterbox and clamp coordinates back to original image pixels.

The code expects the fixed 640×640 export in this package. It is intentionally
independent of the robot's ROS setup or how the edge laptop exchanges data.

## Validation reported by training laptop

The following are the experiment's **observed** metrics, not guaranteed robot
performance. Refer to `metrics.json` for exact figures.

- Original 753-image held-out test split: fine-tuned model P=1.000, R=1.000,
  mAP@0.5=0.995, mAP@0.5:0.95=0.917 (591 annotated instances).
- Real-photo 9-image preliminary holdout: fine-tuned PyTorch P=0.990,
  R=1.000, mAP@0.5=0.995, mAP@0.5:0.95=0.945 (11 instances).
- Same real-photo holdout, ONNX CPU P=0.991, R=1.000,
  mAP@0.5=0.995, mAP@0.5:0.95=0.957 (11 instances).

**Limitations:** Real-photo holdout is small and contains similar images from
one photo session. Original dataset may include near-duplicate frames across
splits. No MentorPi camera / Raspberry Pi 5 throughput was verified. The ONNX
batch-one and PyTorch batch-eight validation configurations differ.

## Hand-off checklist for robot teammate

1. Confirm the UGV camera provides OpenCV BGR images.
2. Verify Python / ONNX Runtime compatibility on the **target** device.
3. Compare 640×640 ONNX predictions with the laptop's example outputs.
4. Measure runtime speed and detection reliability in representative scenes.
5. Calibrate / transform detections into spatial obstacle estimates, fuse with
   geometry and occupancy mapping, apply safety margins, and only then use A*.
6. Keep the original vendor software and weights backed up; do not overwrite.

For robot-safety reasons, this handoff intentionally sends **no velocity
commands** and **does not implement A***.
