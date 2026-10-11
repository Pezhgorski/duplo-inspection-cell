"""SPIKE (throwaway): find the brick, rotate it to a canonical angle, crop a fixed-size patch.

Locate -> align -> inspect: the anomaly model only ever sees the aligned crop, never the background.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

CROP = 240          # output patch size in px: the brick (~170 px long at 36 cm) plus a small margin
MIN_AREA = 2500     # px, smaller blobs are dust/noise
MAX_AREA = 60000


@dataclass
class Located:
    center: tuple
    size: tuple       # (long, short) side in px
    angle: float      # degrees that rotate the long side to horizontal
    contour: np.ndarray


def part_mask(bgr):
    """Pixels that are not white paper: saturated (colored plastic) or clearly darker."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    mask = ((s > 70) | (v < 110)).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def load_plate_roi(path=Path(__file__).resolve().parent.parent / "config" / "plate_roi.json"):
    return json.loads(path.read_text())


def roi_pixels(roi, width, height):
    bw, bh = int(width * roi["w"]), int(height * roi["h"])
    x0, y0 = int(width * roi["cx"] - bw / 2), int(height * roi["cy"] - bh / 2)
    return max(x0, 0), max(y0, 0), min(x0 + bw, width), min(y0 + bh, height)


def mask_outside(bgr, roi):
    """Paint everything outside the plate box with the paper color, so nothing there is ever seen."""
    h, w = bgr.shape[:2]
    x0, y0, x1, y1 = roi_pixels(roi, w, h)
    inside = bgr[y0:y1, x0:x1].reshape(-1, 3)
    paper = np.median(inside[inside.mean(axis=1) > np.percentile(inside.mean(axis=1), 40)], axis=0)
    out = np.empty_like(bgr)
    out[:] = paper.astype(np.uint8)
    out[y0:y1, x0:x1] = bgr[y0:y1, x0:x1]
    return out


def locate(bgr, roi=None):
    """Return the part closest to the image (or plate box) center, ignoring blobs touching the
    border. With a roi, only parts fully inside the plate box count."""
    h, w = bgr.shape[:2]
    if roi is not None:
        x0, y0, x1, y1 = roi_pixels(roi, w, h)
        inner = locate(bgr[y0:y1, x0:x1])  # part touching the box edge -> not located
        if inner is None:
            return None
        inner.center = (inner.center[0] + x0, inner.center[1] + y0)
        return inner
    # CCOMP, not EXTERNAL: the background can form a ring around the paper, and the part then
    # sits inside that ring's hole - EXTERNAL would never return it
    contours, hierarchy = cv2.findContours(part_mask(bgr), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    best, best_dist = None, float("inf")
    for c, (_, _, _, parent) in zip(contours, hierarchy[0] if hierarchy is not None else []):
        if parent != -1:  # hole boundary, not a component outline
            continue
        area = cv2.contourArea(c)
        x, y, cw, ch = cv2.boundingRect(c)
        if not MIN_AREA < area < MAX_AREA or x <= 2 or y <= 2 or x + cw >= w - 2 or y + ch >= h - 2:
            continue
        (cx, cy), (rw, rh), angle = cv2.minAreaRect(c)
        dist = np.hypot(cx - w / 2, cy - h / 2)
        if dist < best_dist:
            if rw < rh:  # make the first side the long one
                rw, rh, angle = rh, rw, angle + 90
            best, best_dist = Located((cx, cy), (rw, rh), angle, c), dist
    return best


def aligned_crop(bgr, loc, size=CROP):
    """Rotate the image around the part so its long side is horizontal and cut a size x size patch."""
    m = cv2.getRotationMatrix2D(loc.center, loc.angle, 1.0)
    m[0, 2] += size / 2 - loc.center[0]
    m[1, 2] += size / 2 - loc.center[1]
    return cv2.warpAffine(bgr, m, (size, size), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


if __name__ == "__main__":
    data = Path(__file__).resolve().parent.parent / "data"
    for cls in ("good", "defect"):
        for p in sorted((data / cls).glob("*.png")):
            loc = locate(cv2.imread(str(p)))
            if loc is None:
                print(f"{cls}/{p.name}: NOT FOUND")
            else:
                print(f"{cls}/{p.name}: {loc.size[0]:.0f}x{loc.size[1]:.0f}px angle {loc.angle:.0f}")
