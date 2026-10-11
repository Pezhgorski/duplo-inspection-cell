"""SPIKE: build the aligned-crop dataset. Every 4th located good image is held out for testing.

Only parts fully inside the plate box (config/plate_roi.json) are used; everything outside the
box is painted paper-colored before cropping, so the model never sees the surroundings.
"""
import shutil
from pathlib import Path

import cv2
import numpy as np

from locate import aligned_crop, load_plate_roi, locate, mask_outside

ROOT = Path(__file__).resolve().parent
DATA = ROOT.parent / "data" / "session1"
OUT = ROOT / "crops"


def located_crops(cls, roi):
    crops, skipped = [], []
    for p in sorted((DATA / cls).glob("*.png")):
        img = cv2.imread(str(p))
        loc = locate(img, roi)
        if loc is None:
            skipped.append(p.stem)
        else:
            crops.append((p.name, aligned_crop(mask_outside(img, roi), loc)))
    return crops, skipped


def main():
    shutil.rmtree(OUT, ignore_errors=True)
    for d in ("train/good", "test/good", "test/defect"):
        (OUT / d).mkdir(parents=True)
    roi = load_plate_roi()

    good, good_skipped = located_crops("good", roi)
    for i, (name, crop) in enumerate(good):
        cv2.imwrite(str(OUT / ("test/good" if i % 4 == 3 else "train/good") / name), crop)
    defect, defect_skipped = located_crops("defect", roi)
    for name, crop in defect:
        cv2.imwrite(str(OUT / "test/defect" / name), crop)

    for d in ("train/good", "test/good", "test/defect"):
        print(f"{d}: {len(list((OUT / d).glob('*.png')))}")
    print("good not fully inside the plate box (skipped):", good_skipped)
    print("defect not located (skipped):", defect_skipped)

    # contact sheet for eyeballing
    tiles = []
    for d in ("train/good", "test/good", "test/defect"):
        for p in sorted((OUT / d).glob("*.png")):
            t = cv2.resize(cv2.imread(str(p)), (160, 160))
            cv2.putText(t, f"{d.split('/')[-1][0]}{p.stem[-2:]}", (4, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            tiles.append(t)
    while len(tiles) % 10:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + 10]) for i in range(0, len(tiles), 10)])
    cv2.imwrite(str(ROOT / "crops_sheet.jpg"), sheet)


if __name__ == "__main__":
    main()
