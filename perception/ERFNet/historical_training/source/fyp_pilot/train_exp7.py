import sys
import os
import json
import random
from pathlib import Path
from datetime import datetime

import numpy as np
import torch

ROOT = Path.home() / "FYP_SLAM"
ERFNET = ROOT / "ERFNet"
PILOT = ERFNET / "fyp_pilot"

sys.path.insert(0, str(ERFNET / "train"))
sys.path.insert(0, str(PILOT))

from erfnet import Net
from fyp_dataset_exp6 import FYPDatasetExp6
from fyp_dataset_exp7 import TelloDatasetExp7

# ==========================================
# EXPERIMENT 7 CONFIGURATION
# ==========================================

SEED = 42
EPOCHS = 6
LR = 0.00003
NUM_CLASSES = 3
WEIGHTS = [1.0, 1.0, 4.0]

torch.manual_seed(SEED)
np.random.seed(SEED)
random.seed(SEED)
torch.set_num_threads(min(4, os.cpu_count() or 1))

rng = torch.Generator().manual_seed(SEED)

# ==========================================
# DATASETS
# ==========================================

phone_train = FYPDatasetExp6("train")
tello_train = TelloDatasetExp7()
phone_val = FYPDatasetExp6("val")

assert len(phone_train) == 14
assert len(tello_train) == 6
assert len(phone_val) == 4

# ==========================================
# LOAD EXPERIMENT 6 MODEL
# ==========================================

exp6_checkpoint = (
    ROOT / "results/ENV1/ERFNet_Experiment6"
    / "20261020_222704/erfnet_exp6_best.pth"
)

checkpoint = torch.load(
    exp6_checkpoint,
    map_location="cpu",
    weights_only=True
)

model = Net(NUM_CLASSES)
model.load_state_dict(
    checkpoint["model_state_dict"],
    strict=True
)

criterion = torch.nn.CrossEntropyLoss(
    weight=torch.tensor(WEIGHTS),
    ignore_index=255
)

optimizer = torch.optim.AdamW(
    model.parameters(),
    lr=LR
)

# ==========================================
# SEPARATE OUTPUT DIRECTORY
# ==========================================

run_name = datetime.now().strftime("%Y%m%d_%H%M%S")

output = (
    ROOT / "results/ENV1/ERFNet_Experiment7"
    / run_name
)

output.mkdir(parents=True, exist_ok=False)

print("===== ERFNET EXPERIMENT 7 =====")
print("Device: CPU")
print("Starting checkpoint: Experiment 6 best")
print("Training phone images:", len(phone_train))
print("Training Tello images:", len(tello_train))
print("Phone validation images:", len(phone_val))
print("Epochs:", EPOCHS)
print("Learning rate:", LR)
print("Class weights:", WEIGHTS)
print("Tello Video 2: excluded")
print("Output:", output, flush=True)

history = []
best_phone_miou = -1.0

# ==========================================
# TRAINING
# ==========================================

for epoch in range(1, EPOCHS + 1):
    model.train()

    # Every phone image is used once per epoch.
    phone_indices = torch.randperm(
        len(phone_train),
        generator=rng
    ).tolist()

    # Tello frames are repeated to provide an equal
    # number of phone and Tello optimization steps.
    tello_indices = []

    while len(tello_indices) < len(phone_indices):
        tello_indices.extend(
            torch.randperm(
                len(tello_train),
                generator=rng
            ).tolist()
        )

    tello_indices = tello_indices[:len(phone_indices)]

    schedule = []

    for p, t in zip(phone_indices, tello_indices):
        schedule.append(("phone", p))
        schedule.append(("tello", t))

    phone_losses = []
    tello_losses = []

    print(
        f"\n===== EPOCH {epoch}/{EPOCHS} =====",
        flush=True
    )

    for step, (source, index) in enumerate(schedule, 1):

        dataset = (
            phone_train if source == "phone"
            else tello_train
        )

        image, mask = dataset[index]

        image = image.unsqueeze(0)
        mask = mask.unsqueeze(0)

        optimizer.zero_grad(set_to_none=True)

        output_logits = model(image)

        loss = criterion(output_logits, mask)

        if not torch.isfinite(loss):
            raise RuntimeError("Non-finite training loss")

        loss.backward()
        optimizer.step()

        if source == "phone":
            phone_losses.append(loss.item())
        else:
            tello_losses.append(loss.item())

        if step == 1 or step % 7 == 0:
            print(
                f"Step {step:02d}/{len(schedule)} "
                f"| {source} "
                f"| Loss={loss.item():.5f}",
                flush=True
            )

    # ======================================
    # PHONE VALIDATION
    # ======================================

    model.eval()

    confusion = torch.zeros(
        (NUM_CLASSES, NUM_CLASSES),
        dtype=torch.int64
    )

    val_losses = []

    with torch.inference_mode():

        for i in range(len(phone_val)):
            image, mask = phone_val[i]

            image = image.unsqueeze(0)
            mask = mask.unsqueeze(0)

            logits = model(image)

            val_loss = criterion(logits, mask)
            val_losses.append(val_loss.item())

            predictions = logits.argmax(dim=1)

            valid = mask != 255

            target = mask[valid]
            predicted = predictions[valid]

            counts = torch.bincount(
                target * NUM_CLASSES + predicted,
                minlength=NUM_CLASSES * NUM_CLASSES
            )

            confusion += counts.reshape(
                NUM_CLASSES, NUM_CLASSES
            )

    class_ious = []

    for class_id in range(NUM_CLASSES):
        tp = confusion[class_id, class_id].item()
        actual = confusion[class_id, :].sum().item()
        predicted = confusion[:, class_id].sum().item()

        union = actual + predicted - tp

        if union == 0:
            raise RuntimeError(
                f"No validation pixels for class {class_id}"
            )

        class_ious.append(tp / union)

    miou = sum(class_ious) / NUM_CLASSES

    record = {
        "epoch": epoch,
        "phone_train_loss": float(np.mean(phone_losses)),
        "tello_train_loss": float(np.mean(tello_losses)),
        "phone_val_loss": float(np.mean(val_losses)),
        "phone_val_miou": miou,
        "phone_val_class_ious": class_ious
    }

    history.append(record)

    print(
        f"Phone train loss: {record['phone_train_loss']:.5f}"
    )
    print(
        f"Tello train loss: {record['tello_train_loss']:.5f}"
    )
    print(
        f"Phone validation mIoU: {miou * 100:.2f}%"
    )
    print(
        "Phone validation class IoUs:",
        [round(v * 100, 2) for v in class_ious]
    )

    save_data = {
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "epoch": epoch,
        "num_classes": NUM_CLASSES,
        "class_weights": WEIGHTS,
        "input_size": 640,
        "phone_val_miou": miou,
        "phone_val_class_ious": class_ious,
        "starting_checkpoint": str(exp6_checkpoint),
        "experiment": "Experiment 7"
    }

    # Fixed final-epoch candidate for Tello evaluation.
    torch.save(
        save_data,
        output / "erfnet_exp7_last.pth"
    )

    # Selected using phone validation only.
    if miou > best_phone_miou:
        best_phone_miou = miou

        torch.save(
            save_data,
            output / "erfnet_exp7_best_phone_val.pth"
        )

        print("NEW BEST PHONE VALIDATION MODEL!")

    with (output / "training_history.json").open("w") as f:
        json.dump(history, f, indent=2)

    print(
        f"Epoch {epoch} completed. "
        f"Best phone validation mIoU: "
        f"{best_phone_miou * 100:.2f}%",
        flush=True
    )

print("\n===== EXPERIMENT 7 COMPLETED =====")
print("Final epoch:", EPOCHS)
print("Results:", output)
print("Last checkpoint: erfnet_exp7_last.pth")
print("Best phone checkpoint: erfnet_exp7_best_phone_val.pth")
print("Tello Video 2 was not used.")
print("Experiment 6 remains unchanged.")
