from pathlib import Path
import sys
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageOps

root = Path.home() / "FYP_SLAM"
erfnet = root / "ERFNet"

sys.path.insert(0, str(erfnet / "train"))
from erfnet import Net

run = root / "results/ENV1/ERFNet_Experiment7/20261020_235553"
dataset = root / "datasets/ENV1/semantic_annotation"
output = run / "tello_video2_visual_diagnostics"

if output.exists():
    raise FileExistsError(f"Output already exists: {output}")

checkpoint = torch.load(
    run / "erfnet_exp7_last.pth",
    map_location="cpu",
    weights_only=True
)

torch.set_num_threads(4)

model = Net(3)
model.load_state_dict(checkpoint["model_state_dict"])
model.eval()

names = [
    "video2_02s",
    "video2_11s",
    "video2_05s",
    "video2_17s"
]

palette = np.array([
    [0, 200, 0],
    [255, 165, 0],
    [255, 0, 0]
], dtype=np.uint8)

def make_overlay(original, labels, ignore=False):
    rgb = np.asarray(original, dtype=np.uint8)
    colored = palette[np.clip(labels, 0, 2)]

    result = rgb.copy()
    valid = (labels != 255) if ignore else np.ones(
        labels.shape, dtype=bool
    )

    result[valid] = (
        0.55 * rgb[valid] + 0.45 * colored[valid]
    ).astype(np.uint8)

    return Image.fromarray(result)

sheet = Image.new("RGB", (1380, 1280), "white")
draw = ImageDraw.Draw(sheet)

output.mkdir(parents=True)

with torch.inference_mode():
    for row, name in enumerate(names):
        with Image.open(dataset / "images" / (name + ".png")) as im:
            original = ImageOps.exif_transpose(im).convert("RGB")

        with Image.open(dataset / "masks" / (name + ".png")) as m:
            ground_truth = np.asarray(m.convert("L"))

        width, height = original.size
        if ground_truth.shape != (height, width):
            raise ValueError(f"Image/mask mismatch: {name}")

        scale = min(640 / width, 640 / height)
        nw = round(width * scale)
        nh = round(height * scale)

        resized = original.resize(
            (nw, nh), Image.Resampling.BILINEAR
        )

        padded = Image.new("RGB", (640, 640), (127, 127, 127))
        x = (640 - nw) // 2
        y = (640 - nh) // 2
        padded.paste(resized, (x, y))

        array = np.asarray(padded, dtype=np.float32) / 255.0
        tensor = torch.from_numpy(
            array
        ).permute(2, 0, 1).unsqueeze(0)

        prediction = model(tensor).argmax(1)[0].byte().numpy()

        prediction = Image.fromarray(prediction)
        prediction = prediction.crop((x, y, x + nw, y + nh))
        prediction = prediction.resize(
            original.size, Image.Resampling.NEAREST
        )

        panels = [
            original,
            make_overlay(original, ground_truth, ignore=True),
            make_overlay(original, np.asarray(prediction))
        ]

        titles = [
            "Original",
            "Ground truth",
            "ERFNet prediction"
        ]

        for col, (panel, title) in enumerate(zip(panels, titles)):
            panel.thumbnail((445, 270))
            px = col * 460 + (460 - panel.width) // 2
            py = row * 320 + 32

            sheet.paste(panel, (px, py))
            draw.text(
                (col * 460 + 10, row * 320 + 8),
                f"{name} - {title}",
                fill="black"
            )

        print("PROCESSED:", name)

contact = output / "TELLO_COMPARISON.jpg"
sheet.save(contact, quality=95)

print("\nSUCCESS: Tello visual comparison created!")
print("Contact sheet:", contact)
print("Checkpoint and original data unchanged.")
