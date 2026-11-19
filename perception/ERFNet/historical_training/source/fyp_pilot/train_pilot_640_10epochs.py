import sys
import os
from pathlib import Path
from datetime import datetime

import torch
from torch.utils.data import DataLoader

ROOT = Path.home() / "FYP_SLAM"
ERFNET = ROOT / "ERFNet"
sys.path.insert(0, str(ERFNET / "train"))

from erfnet import Net
from fyp_dataset_640 import FYPDataset

# =========================================
# 1. CONFIGURATION
# =========================================

torch.manual_seed(42)
torch.set_num_threads(min(4, os.cpu_count() or 1))

DEVICE = torch.device("cpu")
NUM_CLASSES = 3
EPOCHS = 10
LEARNING_RATE = 0.0001

print("===== FYP ERFNET PILOT TRAINING =====")
print("Device:", DEVICE)
print("Classes:", NUM_CLASSES)
print("Epochs:", EPOCHS)

# =========================================
# 2. LOAD DATASET
# =========================================

train_data = FYPDataset("train")
val_data = FYPDataset("val")

assert len(train_data) == 6
assert len(val_data) == 6

train_loader = DataLoader(
    train_data,
    batch_size=1,
    shuffle=True,
    num_workers=0
)

val_loader = DataLoader(
    val_data,
    batch_size=1,
    shuffle=False,
    num_workers=0
)

print("Training images:", len(train_data))
print("Validation images:", len(val_data))

# =========================================
# 3. CREATE THREE-CLASS ERFNET
# =========================================

model = Net(NUM_CLASSES).to(DEVICE)

checkpoint_path = (
    ERFNET / "trained_models/erfnet_pretrained.pth"
)

checkpoint = torch.load(
    checkpoint_path,
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
    k: v for k, v in checkpoint.items()
    if k in model_state
    and v.shape == model_state[k].shape
}

result = model.load_state_dict(
    compatible,
    strict=False
)

assert len(compatible) == 302
assert not result.unexpected_keys

print("Pretrained entries loaded:", len(compatible))
print("New/unloaded entries:", result.missing_keys)

# =========================================
# 4. LOSS AND OPTIMIZER
# =========================================

# Conservative pilot weights, not final tuned weights
class_weights = torch.tensor(
    [1.0, 1.0, 1.0],
    dtype=torch.float32,
    device=DEVICE
)

criterion = torch.nn.CrossEntropyLoss(
    weight=class_weights,
    ignore_index=255
)

optimizer = torch.optim.AdamW(
    model.parameters(),
    lr=LEARNING_RATE
)

# =========================================
# 5. TRAINING
# =========================================

for epoch in range(EPOCHS):
    model.train()
    total_loss = 0.0

    print(f"\n===== EPOCH {epoch + 1}/{EPOCHS} =====")

    for step, (images, masks) in enumerate(
        train_loader, 1
    ):
        images = images.to(DEVICE)
        masks = masks.to(DEVICE)

        optimizer.zero_grad(set_to_none=True)

        outputs = model(images)

        assert outputs.shape[1] == NUM_CLASSES
        assert outputs.shape[-2:] == masks.shape[-2:]

        loss = criterion(outputs, masks)

        if not torch.isfinite(loss):
            raise RuntimeError("Non-finite training loss")

        loss.backward()
        optimizer.step()

        total_loss += loss.item()

        print(
            f"Training {step}/{len(train_loader)} "
            f"| Loss: {loss.item():.5f}",
            flush=True
        )

    average_loss = total_loss / len(train_loader)

    print(f"Average training loss: {average_loss:.5f}")

# =========================================
# 6. VALIDATION
# =========================================

model.eval()

confusion = torch.zeros(
    (NUM_CLASSES, NUM_CLASSES),
    dtype=torch.int64
)

validation_loss = 0.0

with torch.inference_mode():
    for images, masks in val_loader:
        images = images.to(DEVICE)
        masks = masks.to(DEVICE)

        outputs = model(images)
        loss = criterion(outputs, masks)

        validation_loss += loss.item()

        predictions = outputs.argmax(dim=1)

        valid = masks != 255

        targets = masks[valid]
        predictions = predictions[valid]

        indices = targets * NUM_CLASSES + predictions

        counts = torch.bincount(
            indices,
            minlength=NUM_CLASSES * NUM_CLASSES
        )

        confusion += counts.reshape(
            NUM_CLASSES, NUM_CLASSES
        ).cpu()

validation_loss /= len(val_loader)

print("\n===== VALIDATION RESULTS =====")
print(f"Validation loss: {validation_loss:.5f}")

class_names = [
    "Free Space",
    "Blocked",
    "Obstacle"
]

ious = []

for class_id, name in enumerate(class_names):
    tp = confusion[class_id, class_id].item()
    actual = confusion[class_id, :].sum().item()
    predicted = confusion[:, class_id].sum().item()

    union = actual + predicted - tp

    if union > 0:
        iou = tp / union
        ious.append(iou)
        print(f"{name} IoU: {iou:.4f}")
    else:
        print(f"{name} IoU: N/A")

if ious:
    print(f"Mean IoU: {sum(ious) / len(ious):.4f}")

# =========================================
# 7. SAVE NEW PILOT CHECKPOINT
# =========================================

output_root = ROOT / "results/ENV1/ERFNet_pilot_640_10epochs"
run_name = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = output_root / run_name
output_dir.mkdir(parents=True, exist_ok=False)

output_path = output_dir / "erfnet_3class_pilot.pth"

torch.save(
    {
        "model_state_dict": model.state_dict(),
        "num_classes": NUM_CLASSES,
        "epochs": EPOCHS,
        "class_weights": class_weights.cpu(),
        "train_loss": average_loss,
        "validation_loss": validation_loss,
        "confusion_matrix": confusion,
    },
    output_path
)

print("\nModel saved to:", output_path)
print("SUCCESS: First ERFNet training run completed!")
