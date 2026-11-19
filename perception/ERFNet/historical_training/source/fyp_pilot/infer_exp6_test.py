import sys
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageOps, ImageDraw

ROOT = Path.home() / "FYP_SLAM"
ERFNET = ROOT / "ERFNet"

sys.path.insert(0, str(ERFNET / "train"))
from erfnet import Net

run_dir = (
    ROOT / "results/ENV1/ERFNet_Experiment6"
    / "20261020_222704"
)

checkpoint_path = run_dir / "erfnet_exp6_best.pth"

test_dir = (
    ROOT / "datasets/ERFNet_Annotation_Pilot/test"
)

output_dir = run_dir / "unseen_ENV04_predictions"

if not checkpoint_path.is_file():
    raise FileNotFoundError(checkpoint_path)

images = sorted(test_dir.glob("*.jpeg"))

if len(images) != 5:
    raise ValueError(f"Expected 5 test images, found {len(images)}")

if output_dir.exists():
    raise FileExistsError(
        f"Output folder already exists: {output_dir}"
    )

torch.set_num_threads(4)

checkpoint = torch.load(
    checkpoint_path,
    map_location="cpu",
    weights_only=True
)

model = Net(3)
model.load_state_dict(
    checkpoint["model_state_dict"],
    strict=True
)
model.eval()

SIZE = 640

colors = np.array([
    [0, 200, 0],
    [255, 165, 0],
    [255, 0, 0]
], dtype=np.uint8)

output_dir.mkdir(parents=True)

print("===== EXPERIMENT 6 UNSEEN TEST =====")
print("Checkpoint epoch:", checkpoint["epoch"])
print("Validation mIoU:", checkpoint["mean_iou"])
print("Unseen photographs:", len(images))

previews = []

with torch.inference_mode():

    for image_path in images:

        with Image.open(image_path) as img:
            original = ImageOps.exif_transpose(
                img
            ).convert("RGB")

        width, height = original.size

        scale = min(
            SIZE / width,
            SIZE / height
        )

        new_w = round(width * scale)
        new_h = round(height * scale)

        resized = original.resize(
            (new_w, new_h),
            Image.Resampling.BILINEAR
        )

        padded = Image.new(
            "RGB",
            (SIZE, SIZE),
            (127, 127, 127)
        )

        x = (SIZE - new_w) // 2
        y = (SIZE - new_h) // 2

        padded.paste(resized, (x, y))

        array = np.array(
            padded,
            dtype=np.float32
        ) / 255.0

        tensor = torch.from_numpy(
            array
        ).permute(2, 0, 1).unsqueeze(0)

        outputs = model(tensor)

        prediction = outputs.argmax(
            dim=1
        )[0].byte().numpy()

        mask = Image.fromarray(prediction)

        # Remove letterbox padding
        mask = mask.crop((
            x, y,
            x + new_w,
            y + new_h
        ))

        # Restore original image dimensions
        mask = mask.resize(
            (width, height),
            Image.Resampling.NEAREST
        )

        mask_array = np.array(mask)
        colored = Image.fromarray(
            colors[mask_array]
        )

        overlay = Image.blend(
            original,
            colored,
            alpha=0.45
        )

        mask_path = (
            output_dir /
            f"{image_path.stem}_prediction.png"
        )

        overlay_path = (
            output_dir /
            f"{image_path.stem}_overlay.jpg"
        )

        mask.save(mask_path)
        overlay.save(overlay_path, quality=92)

        preview = overlay.copy()
        preview.thumbnail((460, 300))
        previews.append(
            (image_path.stem, preview)
        )

        print("GENERATED:", image_path.name)

# Contact sheet
sheet = Image.new(
    "RGB", (960, 1050), "white"
)
draw = ImageDraw.Draw(sheet)

for i, (name, preview) in enumerate(previews):
    x = (i % 2) * 480
    y = (i // 2) * 350

    sheet.paste(
        preview,
        (x + (480 - preview.width)//2, y + 5)
    )

    draw.text(
        (x + 10, y + 310),
        name,
        fill="black"
    )

contact = output_dir / "ENV04_ALL_5_PREDICTIONS.jpg"
sheet.save(contact, quality=95)

print("\nSUCCESS: Five unseen predictions generated.")
print("Output:", output_dir)
print("Contact sheet:", contact)
print("Original images unchanged.")
