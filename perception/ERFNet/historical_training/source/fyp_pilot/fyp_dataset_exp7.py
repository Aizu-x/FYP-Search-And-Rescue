
from pathlib import Path
import numpy as np
import torch
from PIL import Image, ImageOps
from torch.utils.data import Dataset

class TelloDatasetExp7(Dataset):
    def __init__(self, size=640):
        self.base = (
            Path.home() /
            "FYP_SLAM/datasets/ENV1/semantic_annotation"
        )

        self.size = size

        # Only Video 1 is used for fine-tuning.
        self.images = sorted(
            (self.base / "images").glob("video1_*.png")
        )

        if len(self.images) != 6:
            raise ValueError(
                f"Expected 6 Tello training frames, "
                f"found {len(self.images)}"
            )

        for image_path in self.images:
            mask_path = self.base / "masks" / image_path.name
            if not mask_path.exists():
                raise FileNotFoundError(mask_path)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        image_path = self.images[index]
        mask_path = self.base / "masks" / image_path.name

        with Image.open(image_path) as im:
            image = ImageOps.exif_transpose(im).convert("RGB")

        with Image.open(mask_path) as m:
            mask = m.convert("L")

        if image.size != mask.size:
            raise ValueError(
                f"Image/mask mismatch: {image_path.name}"
            )

        width, height = image.size

        scale = min(
            self.size / width,
            self.size / height
        )

        new_width = round(width * scale)
        new_height = round(height * scale)

        image = image.resize(
            (new_width, new_height),
            Image.Resampling.BILINEAR
        )

        mask = mask.resize(
            (new_width, new_height),
            Image.Resampling.NEAREST
        )

        padded_image = Image.new(
            "RGB",
            (self.size, self.size),
            (127, 127, 127)
        )

        padded_mask = Image.new(
            "L",
            (self.size, self.size),
            255
        )

        x = (self.size - new_width) // 2
        y = (self.size - new_height) // 2

        padded_image.paste(image, (x, y))
        padded_mask.paste(mask, (x, y))

        image_array = (
            np.asarray(padded_image, dtype=np.float32)
            / 255.0
        )

        mask_array = np.asarray(
            padded_mask,
            dtype=np.int64
        )

        found = set(np.unique(mask_array).tolist())

        if not found.issubset({0, 1, 2, 255}):
            raise ValueError(
                f"Invalid mask values: {found}"
            )

        image_tensor = torch.from_numpy(
            image_array.copy()
        ).permute(2, 0, 1)

        mask_tensor = torch.from_numpy(
            mask_array.copy()
        )

        return image_tensor, mask_tensor
