import os
import sys
import json
import random
from pathlib import Path
from datetime import datetime

import numpy as np
import torch
from torch.utils.data import DataLoader

ROOT = Path.home() / "FYP_SLAM"
ERFNET = ROOT / "ERFNet"

sys.path.insert(0, str(ERFNET / "train"))
sys.path.insert(0, str(ERFNET / "fyp_pilot"))

from erfnet import Net
from fyp_dataset_exp6 import FYPDatasetExp6

# ==========================================
# CONFIGURATION
# ==========================================

SEED = 42
NUM_CLASSES = 3
EPOCHS = 10
LEARNING_RATE = 0.0001

CLASS_NAMES = [
    "Free Space",
    "Blocked",
    "Obstacle"
]

WEIGHTS = [1.0, 1.0, 4.0]

torch.manual_seed(SEED)
np.random.seed(SEED)
random.seed(SEED)

torch.set_num_threads(
    min(4, os.cpu_count() or 1)
)

device = torch.device("cpu")

print("===== ERFNET EXPERIMENT 6 =====")
print("Device:", device)
print("Classes:", NUM_CLASSES)
print("Epochs:", EPOCHS)
print("Input resolution: 640 x 640")
print("Class weights:", WEIGHTS)

# ==========================================
# DATASET
# ==========================================

train_dataset = FYPDatasetExp6("train")
val_dataset = FYPDatasetExp6("val")

assert len(train_dataset) == 14
assert len(val_dataset) == 4

generator = torch.Generator()
generator.manual_seed(SEED)

train_loader = DataLoader(
    train_dataset,
    batch_size=1,
    shuffle=True,
    num_workers=0,
    generator=generator
)

val_loader = DataLoader(
    val_dataset,
    batch_size=1,
    shuffle=False,
    num_workers=0
)

print("Training images:", len(train_dataset))
print("Validation images:", len(val_dataset))

# ==========================================
# MODEL AND PRETRAINED WEIGHTS
# ==========================================

model = Net(NUM_CLASSES).to(device)

pretrained_path = (
    ERFNET / "trained_models/erfnet_pretrained.pth"
)

checkpoint = torch.load(
    pretrained_path,
    map_location="cpu",
    weights_only=True
)

if "state_dict" in checkpoint:
    checkpoint = checkpoint["state_dict"]

checkpoint = {
    k.removeprefix("module."): v
    for k, v in checkpoint.items()
    if isinstance(v, torch.Tensor)
}

model_state = model.state_dict()

compatible = {
    k: v
    for k, v in checkpoint.items()
    if k in model_state
    and v.shape == model_state[k].shape
}

result = model.load_state_dict(
    compatible,
    strict=False
)

if len(compatible) != 302:
    raise RuntimeError(
        f"Expected 302 pretrained entries, "
        f"found {len(compatible)}"
    )

if result.unexpected_keys:
    raise RuntimeError(
        f"Unexpected keys: {result.unexpected_keys}"
    )

print("Pretrained parameters loaded:", len(compatible))
print("New/unloaded entries:", result.missing_keys)

# ==========================================
# LOSS AND OPTIMIZER
# ==========================================

class_weights = torch.tensor(
    WEIGHTS,
    dtype=torch.float32,
    device=device
)

criterion = torch.nn.CrossEntropyLoss(
    weight=class_weights,
    ignore_index=255
)

optimizer = torch.optim.AdamW(
    model.parameters(),
    lr=LEARNING_RATE
)

# ==========================================
# OUTPUT DIRECTORY
# ==========================================

run_name = datetime.now().strftime(
    "%Y%m%d_%H%M%S"
)

output_dir = (
    ROOT / "results/ENV1/ERFNet_Experiment6"
    / run_name
)

output_dir.mkdir(
    parents=True,
    exist_ok=False
)

print("Results folder:", output_dir)

# ==========================================
# TRAIN AND VALIDATE
# ==========================================

best_miou = -1.0
best_epoch = 0
history = []

for epoch in range(1, EPOCHS + 1):

    print(
        f"\n========== EPOCH {epoch}/{EPOCHS} ==========",
        flush=True
    )

    model.train()
    train_loss = 0.0

    for step, (images, masks) in enumerate(
        train_loader, start=1
    ):

        images = images.to(device)
        masks = masks.to(device)

        optimizer.zero_grad(set_to_none=True)

        outputs = model(images)

        if outputs.shape != (
            images.shape[0],
            NUM_CLASSES,
            640,
            640
        ):
            raise ValueError(
                f"Unexpected model output: {outputs.shape}"
            )

        loss = criterion(outputs, masks)

        if not torch.isfinite(loss):
            raise RuntimeError("Non-finite training loss")

        loss.backward()
        optimizer.step()

        train_loss += loss.item()

        print(
            f"Train {step:02d}/{len(train_loader)} "
            f"| Loss: {loss.item():.5f}",
            flush=True
        )

    train_loss /= len(train_loader)

    # ======================================
    # VALIDATION
    # ======================================

    model.eval()

    confusion = torch.zeros(
        (NUM_CLASSES, NUM_CLASSES),
        dtype=torch.int64
    )

    val_loss = 0.0

    with torch.inference_mode():

        for images, masks in val_loader:

            images = images.to(device)
            masks = masks.to(device)

            outputs = model(images)

            loss = criterion(outputs, masks)
            val_loss += loss.item()

            predictions = outputs.argmax(dim=1)

            valid = masks != 255

            target = masks[valid]
            predicted = predictions[valid]

            indices = target * NUM_CLASSES + predicted

            counts = torch.bincount(
                indices,
                minlength=NUM_CLASSES * NUM_CLASSES
            )

            confusion += counts.reshape(
                NUM_CLASSES, NUM_CLASSES
            ).cpu()

    val_loss /= len(val_loader)

    # ======================================
    # METRICS
    # ======================================

    ious = []
    precisions = []
    recalls = []

    print("\nValidation results:")

    for class_id, name in enumerate(CLASS_NAMES):

        tp = confusion[class_id, class_id].item()

        actual = confusion[class_id, :].sum().item()
        predicted = confusion[:, class_id].sum().item()

        union = actual + predicted - tp

        iou = tp / union if union > 0 else 0.0
        precision = (
            tp / predicted if predicted > 0 else 0.0
        )
        recall = (
            tp / actual if actual > 0 else 0.0
        )

        ious.append(iou)
        precisions.append(precision)
        recalls.append(recall)

        print(
            f"{name}: "
            f"IoU={iou:.4f} "
            f"Precision={precision:.4f} "
            f"Recall={recall:.4f}"
        )

    miou = sum(ious) / NUM_CLASSES

    print(f"Training loss: {train_loss:.5f}")
    print(f"Validation loss: {val_loss:.5f}")
    print(f"Mean IoU: {miou:.4f}")

    epoch_result = {
        "epoch": epoch,
        "train_loss": train_loss,
        "val_loss": val_loss,
        "mean_iou": miou,
        "class_iou": ious,
        "class_precision": precisions,
        "class_recall": recalls
    }

    history.append(epoch_result)

    # ======================================
    # SAVE CHECKPOINTS
    # ======================================

    save_data = {
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "epoch": epoch,
        "num_classes": NUM_CLASSES,
        "input_size": 640,
        "class_weights": WEIGHTS,
        "train_loss": train_loss,
        "val_loss": val_loss,
        "mean_iou": miou,
        "class_iou": ious,
        "confusion_matrix": confusion,
        "experiment": "Experiment 6"
    }

    # Most recent checkpoint
    torch.save(
        save_data,
        output_dir / "erfnet_exp6_last.pth"
    )

    # Best validation checkpoint
    if miou > best_miou:

        best_miou = miou
        best_epoch = epoch

        torch.save(
            save_data,
            output_dir / "erfnet_exp6_best.pth"
        )

        print(
            "NEW BEST MODEL SAVED!",
            flush=True
        )

    with (
        output_dir / "training_history.json"
    ).open("w") as f:

        json.dump(history, f, indent=2)

    print(
        f"Best epoch so far: {best_epoch} "
        f"| Best mIoU: {best_miou:.4f}",
        flush=True
    )

# ==========================================
# FINAL SUMMARY
# ==========================================

print("\n====================================")
print("EXPERIMENT 6 COMPLETED")
print("====================================")
print("Best epoch:", best_epoch)
print(f"Best validation mIoU: {best_miou:.4f}")
print("Results folder:", output_dir)
print("Best checkpoint: erfnet_exp6_best.pth")
print("Last checkpoint: erfnet_exp6_last.pth")
print("Training history: training_history.json")
print("Previous experiments unchanged.")
