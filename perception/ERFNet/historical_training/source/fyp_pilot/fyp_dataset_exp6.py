
from pathlib import Path
import numpy as np
import torch
from PIL import Image, ImageOps
from torch.utils.data import Dataset

class FYPDatasetExp6(Dataset):
    def __init__(self, split="train", size=640):
        if split not in ("train", "val"):
            raise ValueError("Only train and val allowed")

        self.base = (
            Path.home() /
            "FYP_SLAM/datasets/ERFNet_Annotation_Pilot"
        )

        self.split = split
        self.size = size

        self.images = sorted(
            (self.base / split).glob("*.jpeg")
        )

        expected = 14 if split == "train" else 4

        if len(self.images) != expected:
            raise ValueError(
                f"Expected {expected} {split} images, "
                f"found {len(self.images)}"
            )

        for image_path in self.images:
            mask_path = (
                self.base / "masks" / split /
                (image_path.stem + ".png")
            )

            if not mask_path.exists():
                raise FileNotFoundError(mask_path)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        image_path = self.images[index]

        mask_path = (
            self.base / "masks" / self.split /
            (image_path.stem + ".png")
        )

        with Image.open(image_path) as img:
            image = ImageOps.exif_transpose(img).convert("RGB")

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

        image_array = np.array(
            padded_image,
            dtype=np.float32
        ) / 255.0

        mask_array = np.array(
            padded_mask,
            dtype=np.int64
        )

        valid = {0, 1, 2, 255}
        found = set(np.unique(mask_array).tolist())

        if not found.issubset(valid):
            raise ValueError(
                f"Invalid mask values: {found}"
            )

        image_tensor = torch.from_numpy(
            image_array
        ).permute(2, 0, 1)

        mask_tensor = torch.from_numpy(
            mask_array
        )

        return image_tensor, mask_tensor
