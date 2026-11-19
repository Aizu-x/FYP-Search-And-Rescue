"""Project-specific ERFNet I/O, augmentation and honest held-out evaluation."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import random
import numpy as np
from PIL import Image, ImageOps, ImageEnhance
import torch
from .architecture import Net

LABELS = (0, 1, 2, 255)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def samples(root, split):
    root = Path(root)
    if split.startswith("phone_"):
        part = split.removeprefix("phone_")
        pairs = [(p, root / "training/phone/masks" / part / (p.stem + ".png"))
                 for p in sorted((root / "training/phone" / part).glob("*.jpeg"))]
        expected = {"train": 14, "val": 4, "test": 5}[part]
    elif split in ("tello_train", "tello_diagnostic"):
        video = "video1" if split == "tello_train" else "video2"
        pairs = [(p, root / "training/tello/masks" / p.name)
                 for p in sorted((root / "training/tello/images").glob(video + "_*.png"))]
        expected = 6
    elif split == "physical_three":
        pairs = [(p, root / "actual/groundtruth_masks" / (p.stem + ".png"))
                 for p in sorted((root / "actual/annotations").glob("ENV*__*.jpg"))]
        expected = 3
    else:
        raise ValueError(f"Unknown split {split}")
    if len(pairs) != expected or any(not m.is_file() for _, m in pairs):
        raise ValueError(f"{split}: expected {expected} matched image/mask pairs; got {len(pairs)}")
    return pairs


def read_pair(pair):
    with Image.open(pair[0]) as im:
        image = ImageOps.exif_transpose(im).convert("RGB")
    with Image.open(pair[1]) as im:
        mask = im.copy()  # preserve palette indices rather than RGB luminance
    a = np.asarray(mask)
    if a.ndim != 2 or not set(np.unique(a)).issubset(LABELS):
        raise ValueError(f"Invalid semantic IDs: {pair[1]}")
    mask = Image.fromarray(a.astype(np.uint8))
    if image.size != mask.size:
        raise ValueError(f"Image/mask dimensions disagree: {pair[0]}")
    return image, mask


def augment_pair(image, mask, rng):
    """Training only: preserve scene context on half the presentations."""
    if rng.random() < .5:
        width, height = image.size
        fraction = rng.uniform(.65, .90)
        cw, ch = max(8, round(width * fraction)), max(8, round(height * fraction))
        ys, xs = np.nonzero(np.asarray(mask) == 2)
        if len(xs):
            idx = rng.randrange(len(xs))
            cx, cy = int(xs[idx]), int(ys[idx])
            left = max(0, min(width - cw, cx - cw // 2))
            top = max(0, min(height - ch, cy - ch // 2))
        else:
            left, top = rng.randint(0, width - cw), rng.randint(0, height - ch)
        box = (left, top, left + cw, top + ch)
        image, mask = image.crop(box), mask.crop(box)
    if rng.random() < .5:
        image, mask = ImageOps.mirror(image), ImageOps.mirror(mask)
    angle = rng.uniform(-10, 10)
    image = image.rotate(angle, resample=Image.Resampling.BILINEAR, fillcolor=(127, 127, 127))
    mask = mask.rotate(angle, resample=Image.Resampling.NEAREST, fillcolor=255)
    image = ImageEnhance.Brightness(image).enhance(rng.uniform(.85, 1.15))
    image = ImageEnhance.Contrast(image).enhance(rng.uniform(.85, 1.15))
    image = ImageEnhance.Color(image).enhance(rng.uniform(.9, 1.1))
    return image, mask


def prepare(pair, size=640, rng=None):
    if size <= 0 or size % 8:
        raise ValueError("ERFNet input size must be a positive multiple of 8")
    image, mask = read_pair(pair)
    original_size = image.size
    if rng is not None:
        image, mask = augment_pair(image, mask, rng)
    w, h = image.size
    scale = min(size / w, size / h)
    nw, nh = round(w * scale), round(h * scale)
    image = image.resize((nw, nh), Image.Resampling.BILINEAR)
    mask = mask.resize((nw, nh), Image.Resampling.NEAREST)
    x, y = (size - nw) // 2, (size - nh) // 2
    pi = Image.new("RGB", (size, size), (127, 127, 127))
    pm = Image.new("L", (size, size), 255)
    pi.paste(image, (x, y)); pm.paste(mask, (x, y))
    pixels = np.asarray(pi, np.float32) / 255.
    labels = np.asarray(pm, np.int64).copy()
    return (torch.from_numpy(pixels.copy()).permute(2, 0, 1), torch.from_numpy(labels),
            {"x": x, "y": y, "width": nw, "height": nh, "original_size": original_size})


def confusion_metrics(matrix):
    cm = np.asarray(matrix, dtype=np.int64)
    tp = np.diag(cm)
    actual, predicted = cm.sum(1), cm.sum(0)
    union = actual + predicted - tp
    ratio = lambda a, b: float(a / b) if b else None
    classes = [{"class": name, "iou": ratio(tp[i], union[i]),
                "precision": ratio(tp[i], predicted[i]), "recall": ratio(tp[i], actual[i]),
                "gt_pixels": int(actual[i]), "predicted_pixels": int(predicted[i])}
               for i, name in enumerate(("free_space", "blocked", "obstacle"))]
    occupied_gt = int(cm[1:, :].sum())
    occupied_pred = int(cm[:, 1:].sum())
    occupied_tp = int(cm[1:, 1:].sum())
    return {"confusion_matrix": cm.tolist(), "classes": classes,
            "mean_iou": float(np.mean([c["iou"] for c in classes if c["iou"] is not None])),
            "pixel_accuracy": ratio(int(tp.sum()), int(cm.sum())),
            "occupied_recall": ratio(occupied_tp, occupied_gt),
            "occupied_precision": ratio(occupied_tp, occupied_pred),
            "occupied_pixels_predicted_free": int(cm[1:, 0].sum()),
            "occupied_false_free_rate": ratio(int(cm[1:, 0].sum()), occupied_gt),
            "obstacle_to_free_rate": ratio(int(cm[2, 0]), int(actual[2])),
            "obstacle_to_blocked_rate": ratio(int(cm[2, 1]), int(actual[2])),
            "free_pixels_predicted_occupied": int(cm[0, 1:].sum()),
            "free_false_occupied_rate": ratio(int(cm[0, 1:].sum()), int(actual[0]))}


def load_model(checkpoint, device="cpu"):
    checkpoint = Path(checkpoint)
    data = torch.load(checkpoint, map_location="cpu", weights_only=True)
    if data.get("num_classes", 3) != 3:
        raise ValueError("Expected the fine-tuned THREE-class checkpoint")
    model = Net(3)
    model.load_state_dict(data["model_state_dict"], strict=True)
    model.to(device)
    metadata = {k: v for k, v in data.items() if not k.endswith("state_dict")}
    metadata["checkpoint_sha256"] = sha256(checkpoint)
    return model, metadata


def evaluate(model, pairs, size=640, device="cpu", output=None):
    model.eval()
    total = np.zeros((3, 3), np.int64)
    results = []
    if output is not None:
        Path(output).mkdir(parents=True, exist_ok=False)
    with torch.inference_mode():
        for pair in pairs:
            image, target, geometry = prepare(pair, size)
            logits = model(image.unsqueeze(0).to(device))
            pred = logits.argmax(1)[0].cpu().numpy()
            target = target.numpy()
            valid = target != 255
            cm = np.bincount(target[valid] * 3 + pred[valid], minlength=9).reshape(3, 3)
            total += cm
            results.append({"image": pair[0].name, "metrics": confusion_metrics(cm)})
            if output is not None:
                Image.fromarray(pred.astype(np.uint8)).save(Path(output) / (pair[0].stem + f"_prediction_{size}.png"))
                x, y, w, h = (geometry[k] for k in ("x", "y", "width", "height"))
                full = Image.fromarray(pred[y:y+h, x:x+w].astype(np.uint8)).resize(geometry["original_size"], Image.Resampling.NEAREST)
                full.save(Path(output) / (pair[0].stem + "_prediction_original.png"))
    return {"evaluation_size": size, "image_count": len(pairs),
            "metrics": confusion_metrics(total), "per_image": results}
