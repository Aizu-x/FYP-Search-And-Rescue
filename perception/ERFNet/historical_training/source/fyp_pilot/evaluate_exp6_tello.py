import sys
import os
import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageOps

ROOT = Path.home() / "FYP_SLAM"
ERFNET = ROOT / "ERFNet"

sys.path.insert(0, str(ERFNET / "train"))
from erfnet import Net

RUN = (
    ROOT / "results/ENV1/ERFNet_Experiment6"
    / "20261020_222704"
)

CHECKPOINT = RUN / "erfnet_exp6_best.pth"
DATASET = ROOT / "datasets/ENV1/semantic_annotation"
OUTPUT = RUN / "tello_cross_camera_evaluation"

SIZE = 640
NUM_CLASSES = 3
NAMES = ["Free Space", "Blocked", "Obstacle"]

torch.set_num_threads(min(4, os.cpu_count() or 1))

if OUTPUT.exists():
    raise FileExistsError(
        f"Evaluation output already exists: {OUTPUT}"
    )

files = sorted((DATASET / "images").glob("video*.png"))

if len(files) != 12:
    raise ValueError(
        f"Expected 12 Tello frames, found {len(files)}"
    )

checkpoint = torch.load(
    CHECKPOINT,
    map_location="cpu",
    weights_only=True
)

model = Net(NUM_CLASSES)
model.load_state_dict(
    checkpoint["model_state_dict"],
    strict=True
)
model.eval()

def get_metrics(confusion):
    results = []
    valid_ious = []

    for k, name in enumerate(NAMES):
        tp = int(confusion[k, k])
        actual = int(confusion[k, :].sum())
        predicted = int(confusion[:, k].sum())
        union = actual + predicted - tp

        iou = tp / union if union else None
        precision = tp / predicted if predicted else 0.0
        recall = tp / actual if actual else 0.0

        if iou is not None:
            valid_ious.append(iou)

        results.append({
            "class": name,
            "iou": iou,
            "precision": precision,
            "recall": recall,
            "ground_truth_pixels": actual,
            "predicted_pixels": predicted
        })

    return {
        "classes": results,
        "mean_iou": (
            sum(valid_ious) / len(valid_ious)
            if valid_ious else None
        ),
        "pixel_accuracy": (
            float(np.trace(confusion) / confusion.sum())
            if confusion.sum() else None
        )
    }

total_confusion = np.zeros(
    (NUM_CLASSES, NUM_CLASSES),
    dtype=np.int64
)

per_image = []

print("===== ERFNET EXPERIMENT 6 TELLO EVALUATION =====")
print("Checkpoint:", CHECKPOINT)
print("Checkpoint epoch:", checkpoint["epoch"])
print("Tello images:", len(files))
print("Evaluation resolution: 640 x 640")
print("Ignore class: 255")

with torch.inference_mode():
    for image_path in files:
        mask_path = (
            DATASET / "masks"
            / f"{image_path.stem}.png"
        )

        if not mask_path.exists():
            raise FileNotFoundError(mask_path)

        with Image.open(image_path) as img:
            image = ImageOps.exif_transpose(
                img
            ).convert("RGB")

        with Image.open(mask_path) as m:
            mask = m.convert("L")

        if image.size != mask.size:
            raise ValueError(
                f"Image/mask size mismatch: {image_path.name}"
            )

        width, height = image.size
        scale = min(SIZE / width, SIZE / height)

        new_w = round(width * scale)
        new_h = round(height * scale)

        image = image.resize(
            (new_w, new_h),
            Image.Resampling.BILINEAR
        )

        mask = mask.resize(
            (new_w, new_h),
            Image.Resampling.NEAREST
        )

        x = (SIZE - new_w) // 2
        y = (SIZE - new_h) // 2

        padded_image = Image.new(
            "RGB", (SIZE, SIZE), (127, 127, 127)
        )

        padded_mask = Image.new(
            "L", (SIZE, SIZE), 255
        )

        padded_image.paste(image, (x, y))
        padded_mask.paste(mask, (x, y))

        image_array = (
            np.asarray(
                padded_image,
                dtype=np.float32
            ) / 255.0
        )

        ground_truth = np.asarray(
            padded_mask,
            dtype=np.int64
        )

        allowed = {0, 1, 2, 255}
        found = set(np.unique(ground_truth).tolist())

        if not found.issubset(allowed):
            raise ValueError(
                f"Unexpected mask values: {found}"
            )

        tensor = torch.from_numpy(
            image_array
        ).permute(2, 0, 1).unsqueeze(0)

        outputs = model(tensor)

        prediction = outputs.argmax(
            dim=1
        )[0].numpy()

        valid = ground_truth != 255

        target_pixels = ground_truth[valid]
        predicted_pixels = prediction[valid]

        counts = np.bincount(
            target_pixels * NUM_CLASSES
            + predicted_pixels,
            minlength=NUM_CLASSES * NUM_CLASSES
        )

        confusion = counts.reshape(
            NUM_CLASSES, NUM_CLASSES
        )

        total_confusion += confusion

        metrics = get_metrics(confusion)

        per_image.append({
            "image": image_path.name,
            "valid_pixels": int(valid.sum()),
            "confusion_matrix": confusion.tolist(),
            "metrics": metrics
        })

        print(
            f"{image_path.stem}: "
            f"mIoU={metrics['mean_iou'] * 100:.2f}%"
        )

results = get_metrics(total_confusion)

print("\n===== FINAL TEST RESULTS =====")

for item in results["classes"]:
    print(f"\n{item['class']}:")
    print(f"  IoU:       {item['iou'] * 100:.2f}%")
    print(f"  Precision: {item['precision'] * 100:.2f}%")
    print(f"  Recall:    {item['recall'] * 100:.2f}%")

print("\nTEST MEAN IoU: "
      f"{results['mean_iou'] * 100:.2f}%")

print("TEST PIXEL ACCURACY: "
      f"{results['pixel_accuracy'] * 100:.2f}%")

print("\n===== CONFUSION MATRIX =====")
print("Rows = ground truth")
print("Columns = predictions")
print(total_confusion)

OUTPUT.mkdir(parents=True, exist_ok=False)

report = {
    "experiment": "ERFNet Experiment 6",
    "checkpoint_epoch": checkpoint["epoch"],
    "evaluation_size": SIZE,
    "ignored_label": 255,
    "class_names": NAMES,
    "overall_metrics": results,
    "confusion_matrix": total_confusion.tolist(),
    "per_image": per_image
}

report_path = OUTPUT / "test_metrics.json"

with report_path.open("w") as f:
    json.dump(report, f, indent=2)

print("\nSUCCESS: Test evaluation completed!")
print("Report:", report_path)
print("Model checkpoint unchanged.")
