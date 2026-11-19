from pathlib import Path
import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

class FYPDataset(Dataset):
    def __init__(self, split="train"):
        self.base = (
            Path.home() /
            "FYP_SLAM/datasets/ENV1/semantic_annotation"
        )

        prefix = "video1_" if split == "train" else "video2_"

        self.images = sorted(
            (self.base / "images").glob(prefix + "*.png")
        )

        self.size = (640, 480)

        for image_path in self.images:
            mask_path = self.base / "masks" / image_path.name
            if not mask_path.exists():
                raise FileNotFoundError(mask_path)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        image_path = self.images[index]
        mask_path = self.base / "masks" / image_path.name

        image = Image.open(image_path).convert("RGB")
        mask = Image.open(mask_path)

        image = image.resize(
            self.size, Image.Resampling.BILINEAR
        )

        mask = mask.resize(
            self.size, Image.Resampling.NEAREST
        )

        image = torch.from_numpy(
            np.array(image, dtype=np.float32)
        ).permute(2, 0, 1) / 255.0

        mask = torch.from_numpy(
            np.array(mask, dtype=np.int64)
        )

        valid_ids = {0, 1, 2, 255}
        found_ids = set(torch.unique(mask).tolist())

        if not found_ids.issubset(valid_ids):
            raise ValueError(
                f"Invalid mask values: {found_ids}"
            )

        return image, mask
