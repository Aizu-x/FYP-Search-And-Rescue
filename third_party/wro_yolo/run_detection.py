"""Run the portable WRO detector on an image or directory of images."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import cv2
from wro_detector import WRODetector, draw_detections

EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}


def main():
    p = argparse.ArgumentParser(description="Two-class red/green WRO obstacle detector")
    p.add_argument("--source", required=True, help="Image path or directory of images")
    p.add_argument("--output", default="outputs", help="Where to save JSON + annotated images")
    p.add_argument("--model", default=None, help="Optional ONNX model path")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--iou", type=float, default=0.45)
    args = p.parse_args()
    source = Path(args.source)
    if source.is_file() and source.suffix.lower() in EXTENSIONS:
        images = [source]
    elif source.is_dir():
        images = sorted(p for p in source.iterdir() if p.is_file() and p.suffix.lower() in EXTENSIONS)
    else:
        p.error("--source must be an image file or a directory of images")
    if not images:
        p.error("No supported images found")
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    detector = WRODetector(args.model, conf_threshold=args.conf, iou_threshold=args.iou)
    all_results = []
    for index, path in enumerate(images, 1):
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Cannot read image: {path}")
        detections = detector.predict(frame)
        entry = {
            "image": str(path),
            "image_width": int(frame.shape[1]),
            "image_height": int(frame.shape[0]),
            "detections": detections,
        }
        all_results.append(entry)
        # A prefix avoids collisions when files from different directories share names.
        prefix = f"{index:03d}_{path.stem}"
        (out / (prefix + ".json")).write_text(json.dumps(entry, indent=2))
        cv2.imwrite(str(out / (prefix + ".jpg")), draw_detections(frame, detections))
        print(f"{path.name}: {len(detections)} obstacle(s)")
    (out / "all_detections.json").write_text(json.dumps(all_results, indent=2))
    print(f"Saved results to: {out.resolve()}")


if __name__ == "__main__":
    main()
