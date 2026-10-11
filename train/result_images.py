"""Result images for the write-up, from the current model (models/meta.json):
- docs/images/results_borderline.jpg: the cases nearest the threshold, photo | heatmap, hottest spot circled
- docs/images/results_all.jpg: every validation crop with its heatmap, sorted by score

Usage:  python train/result_images.py 0018 0040 0010 0032 0072 0073
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from cell.anomaly import AnomalyModel  # noqa: E402
from cell.geometry import find_part, load_plate, part_box  # noqa: E402

CROPS = ROOT / "models" / "crops"
OUT = ROOT / "docs" / "images"
GREEN, RED, WHITE, YELLOW = (0, 160, 0), (0, 0, 220), (255, 255, 255), (0, 255, 255)


def put_label(img, x, y, label: str, good: bool, verdict: str, scale: float, thickness: int):
    """Label + score in the true class color, then the verdict in its own color (NG red, OK green),
    so a wrong decision stands out: e.g. green 'GOOD 18 0.54' followed by a red 'NG'."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, label, (x, y), font, scale, GREEN if good else RED, thickness, cv2.LINE_AA)
    x += cv2.getTextSize(label + " ", font, scale, thickness)[0][0]
    cv2.putText(img, verdict, (x, y), font, scale, GREEN if verdict == "OK" else RED, thickness + 1, cv2.LINE_AA)


def colorize(heat: np.ndarray) -> np.ndarray:
    return cv2.applyColorMap((np.clip(heat, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET)


def main():
    meta = json.loads((ROOT / "models" / "meta.json").read_text())
    model = AnomalyModel(ROOT / meta["model"])
    plate = load_plate()
    session = ROOT / "data" / meta["session"]
    rows = []
    for r in meta["val"]:
        crop = cv2.imread(str(CROPS / "val" / r["label"] / r["name"]))
        part = find_part(cv2.imread(str(session / r["label"] / r["name"])), plate).part
        rows.append({**r, "crop": crop, "heat": model.heatmap(crop), "box": part_box(part)})

    zoom_names = sys.argv[1:]
    panels = []
    size = 480
    for r in sorted((r for r in rows if Path(r["name"]).stem in zoom_names), key=lambda r: r["score"]):
        img = cv2.resize(r["crop"], (size, size), interpolation=cv2.INTER_CUBIC)
        heat = cv2.resize(r["heat"], (size, size))
        s = size / r["crop"].shape[0]
        x0, y0, x1, y1 = (int(v * s) for v in r["box"])
        inside = heat[y0:y1, x0:x1]
        y, x = np.unravel_index(np.argmax(inside), inside.shape)
        overlay = cv2.addWeighted(img, 0.5, colorize(heat), 0.5, 0)
        for view in (img, overlay):
            cv2.rectangle(view, (x0, y0), (x1, y1), WHITE, 1)
            cv2.circle(view, (int(x + x0), int(y + y0)), 36, YELLOW, 3, cv2.LINE_AA)
        good = r["label"] == "good"
        bar = np.full((44, size * 2, 3), 255, np.uint8)
        put_label(bar, 12, 31, f"{'GOOD' if good else 'DEFECT'} {Path(r['name']).stem}  score {r['score']:.3f} ->",
                  good, r["verdict"], 0.85, 2)
        panels.append(np.vstack([bar, np.hstack([img, overlay])]))
    if len(panels) % 2:
        panels.append(np.full_like(panels[0], 255))
    cv2.imwrite(str(OUT / "results_borderline.jpg"),
                np.vstack([np.hstack(panels[i:i + 2]) for i in range(0, len(panels), 2)]),
                [cv2.IMWRITE_JPEG_QUALITY, 90])

    tiles = []
    for r in sorted(rows, key=lambda r: r["score"]):
        t = cv2.addWeighted(cv2.resize(r["crop"], (200, 200)), 0.55, colorize(cv2.resize(r["heat"], (200, 200))), 0.45, 0)
        good = r["label"] == "good"
        # a white bar behind the label: OpenCV 5 draws thick and thin text at different widths
        t[:30] = 255
        put_label(t, 6, 21, f"{'GOOD' if good else 'DEFECT'} {Path(r['name']).stem[-2:]}  {r['score']:.2f}",
                  good, r["verdict"], 0.5, 1)
        tiles.append(t)
    while len(tiles) % 8:
        tiles.append(np.full_like(tiles[0], 255))
    cv2.imwrite(str(OUT / "results_all.jpg"),
                np.vstack([np.hstack(tiles[i:i + 8]) for i in range(0, len(tiles), 8)]),
                [cv2.IMWRITE_JPEG_QUALITY, 88])
    print("wrote", OUT / "results_borderline.jpg", "and", OUT / "results_all.jpg")


if __name__ == "__main__":
    main()
