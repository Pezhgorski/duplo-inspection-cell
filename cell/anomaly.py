"""PatchCore inference on an aligned crop: heatmap + score inside the part-shaped ROI."""
from pathlib import Path

import cv2
import numpy as np
import torch


class AnomalyModel:
    def __init__(self, path: Path):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # our own file written by train/train_model.py, so unpickling it is safe
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        self.model = checkpoint["model"].eval().to(self.device)

    @torch.inference_mode()
    def heatmap(self, crop_bgr: np.ndarray) -> np.ndarray:
        """Anomaly map (0..1, 0.5 = anomalib's own threshold) resized to the crop."""
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float().div(255).unsqueeze(0).to(self.device)
        amap = self.model(tensor).anomaly_map.squeeze().float().cpu().numpy()
        return cv2.resize(amap, (crop_bgr.shape[1], crop_bgr.shape[0]))


def roi_score(heatmap: np.ndarray, box: tuple[int, int, int, int]) -> float:
    x0, y0, x1, y1 = box
    return float(heatmap[y0:y1, x0:x1].max())
