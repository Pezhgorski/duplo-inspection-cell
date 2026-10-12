"""Find the part inside the plate box, rotate it to a canonical angle and crop it.

The plate box (config/plate_roi.json, fitted in tools/capture.py) only decides where to search.
Nothing outside it is painted over - the spike showed the paint seam itself scores as an anomaly.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PLATE_FILE = ROOT / "config" / "plate_roi.json"
CROP_SIZE = 240      # px: a 2x4 brick (~170 px long at ~36 cm) plus margin
MIN_AREA = 2500      # px, smaller blobs are dust/noise
MAX_AREA = 60000     # px, bigger blobs are hands/arm, not parts


@dataclass(frozen=True)
class Part:
    center: tuple[float, float]  # full-image pixels
    length: float                # long side, px
    width: float                 # short side, px
    angle: float                 # degrees that rotate the long side to horizontal


@dataclass(frozen=True)
class Search:
    status: str                  # "found", "empty", "outside" (touches the box edge), "multiple",
                                 # "too_large" (bigger than any part: a hand or the arm)
    part: Part | None = None
    crowded: bool = False        # a too-large blob (a hand?) is in the box besides the found part


def load_plate(path=PLATE_FILE) -> dict:
    return json.loads(Path(path).read_text())


def plate_pixels(plate: dict, width: int, height: int) -> tuple[int, int, int, int]:
    """(x0, y0, x1, y1) of the plate box in pixels."""
    bw, bh = int(width * plate["w"]), int(height * plate["h"])
    x0, y0 = int(width * plate["cx"] - bw / 2), int(height * plate["cy"] - bh / 2)
    return max(x0, 0), max(y0, 0), min(x0 + bw, width), min(y0 + bh, height)


def part_mask(bgr: np.ndarray) -> np.ndarray:
    """Pixels that are not white paper: saturated (colored plastic) or clearly darker."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] > 70) | (hsv[..., 2] < 110)).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def find_part(bgr: np.ndarray, plate: dict) -> Search:
    h, w = bgr.shape[:2]
    x0, y0, x1, y1 = plate_pixels(plate, w, h)
    box = bgr[y0:y1, x0:x1]
    bh, bw = box.shape[:2]
    # CCOMP, not EXTERNAL: a blob can sit inside another blob's hole and EXTERNAL would drop it
    contours, hierarchy = cv2.findContours(part_mask(box), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    inside, touching, too_large = [], False, False
    for contour, (_, _, _, parent) in zip(contours, hierarchy[0] if hierarchy is not None else []):
        area = cv2.contourArea(contour)
        if parent != -1 or area < MIN_AREA:
            continue
        x, y, cw, ch = cv2.boundingRect(contour)
        if area > MAX_AREA:
            too_large = True
        elif x <= 1 or y <= 1 or x + cw >= bw - 1 or y + ch >= bh - 1:
            touching = True
        else:
            inside.append(contour)
    # blobs touching the box edge are either a part sticking out or background creeping in at the
    # edge (e.g. the tubs after a nudge); like oversized blobs, they only matter when there is no
    # complete part inside
    if not inside:
        return Search("too_large" if too_large else "outside" if touching else "empty")
    if len(inside) > 1:
        return Search("multiple", crowded=too_large)
    (cx, cy), (rw, rh), angle = cv2.minAreaRect(inside[0])
    if rw < rh:  # make the first side the long one
        rw, rh, angle = rh, rw, angle + 90
    return Search("found", Part((cx + x0, cy + y0), rw, rh, angle), crowded=too_large)


def aligned_crop(bgr: np.ndarray, part: Part, size: int = CROP_SIZE) -> np.ndarray:
    """Rotate around the part so its long side is horizontal and cut a size x size patch."""
    m = cv2.getRotationMatrix2D(part.center, part.angle, 1.0)
    m[0, 2] += size / 2 - part.center[0]
    m[1, 2] += size / 2 - part.center[1]
    return cv2.warpAffine(bgr, m, (size, size), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def part_box(part: Part, size: int = CROP_SIZE, margin: int = 12) -> tuple[int, int, int, int]:
    """The part-shaped scoring ROI in crop coordinates: the part rectangle plus a margin."""
    half_l, half_w = part.length / 2 + margin, part.width / 2 + margin
    c = size / 2
    return (max(int(c - half_l), 0), max(int(c - half_w), 0),
            min(int(c + half_l), size), min(int(c + half_w), size))
