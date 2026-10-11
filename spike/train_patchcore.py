"""SPIKE: train PatchCore on good crops only, then score held-out good + defect crops."""
import time
from pathlib import Path

import cv2
import numpy as np
from anomalib.data import Folder
from anomalib.engine import Engine
from anomalib.models import Patchcore
from lightning.pytorch import seed_everything

ROOT = Path(__file__).resolve().parent
CROPS = ROOT / "crops"


def main():
    seed_everything(42)  # PatchCore's coreset sampling is random; fix it so the numbers are reproducible
    datamodule = Folder(
        name="duplo",
        root=CROPS,
        normal_dir="train/good",
        normal_test_dir="test/good",
        abnormal_dir="test/defect",
        val_split_mode="same_as_test",
        train_batch_size=8,
        eval_batch_size=8,
        num_workers=0,  # Windows: avoid dataloader worker spawn overhead
    )
    model = Patchcore(coreset_sampling_ratio=0.25)
    engine = Engine(default_root_dir=ROOT / "results", logger=False)

    t = time.time()
    engine.fit(model=model, datamodule=datamodule)
    print(f"\ntraining took {time.time() - t:.1f}s")

    t = time.time()
    batches = engine.predict(model=model, datamodule=datamodule, return_predictions=True)
    n = sum(len(b.image_path) for b in batches)
    print(f"inference: {n} images in {time.time() - t:.1f}s")

    rows = []
    for b in batches:
        for path, score, amap in zip(b.image_path, b.pred_score, b.anomaly_map):
            rows.append((Path(path), float(score), amap.squeeze().cpu().numpy()))
    rows.sort(key=lambda r: r[1])

    print("\nscore  label   file")
    for path, score, _ in rows:
        print(f"{score:.3f}  {path.parent.name:7} {path.name}")
    good = [s for p, s, _ in rows if p.parent.name == "good"]
    defect = [s for p, s, _ in rows if p.parent.name == "defect"]
    print(f"\nmax good {max(good):.3f} | min defect {min(defect):.3f} | "
          f"separable: {max(good) < min(defect)}")

    # ROI scoring: only the region around the (centered, aligned) part counts, not the crop corners
    for width, height in ((0.8, 0.6), (0.75, 0.45), (0.7, 0.4)):
        scored = [(p, roi_score(a, width, height)) for p, _, a in rows]
        good = sorted((s, p.stem) for p, s in scored if p.parent.name == "good")
        defect = sorted((s, p.stem) for p, s in scored if p.parent.name == "defect")
        overlap = [n for s, n in good if s >= defect[0][0]] + [n for s, n in defect if s <= good[-1][0]]
        print(f"ROI {width}x{height}: max good {good[-1][0]:.3f} ({good[-1][1]}) | "
              f"min defect {defect[0][0]:.3f} ({defect[0][1]}) | overlapping: {overlap}")

    save_heatmaps(rows)
    save_zoom(rows, ["0010", "0032", "0044", "0072", "0073", "0068"])


def save_zoom(rows, names, size=480):
    """Large side-by-side views (photo | heatmap) with the hottest spot circled."""
    hi = max(a.max() for _, _, a in rows)
    panels = []
    for path, score, amap in rows:
        if path.stem not in names:
            continue
        img = cv2.resize(cv2.imread(str(path)), (size, size), interpolation=cv2.INTER_CUBIC)
        heat = cv2.resize(amap, (size, size))
        y, x = np.unravel_index(np.argmax(heat), heat.shape)
        overlay = cv2.addWeighted(img, 0.5, cv2.applyColorMap(
            (np.clip(heat / hi, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET), 0.5, 0)
        for view in (img, overlay):
            cv2.circle(view, (int(x), int(y)), 38, (0, 255, 255), 3, cv2.LINE_AA)
        label = f"{'GOOD' if path.parent.name == 'good' else 'DEFECT'} {path.stem}  score {score:.2f}"
        color = (0, 160, 0) if path.parent.name == "good" else (0, 0, 220)
        bar = np.full((44, size * 2, 3), 255, np.uint8)
        cv2.putText(bar, label, (12, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
        panels.append(np.vstack([bar, np.hstack([img, overlay])]))
    panels.sort(key=lambda p: 0)
    pairs = [np.hstack(panels[i:i + 2]) if i + 1 < len(panels) else
             np.hstack([panels[i], np.full_like(panels[i], 255)]) for i in range(0, len(panels), 2)]
    cv2.imwrite(str(ROOT / "zoom_overlap.jpg"), np.vstack(pairs), [cv2.IMWRITE_JPEG_QUALITY, 90])


def roi_score(amap, width=0.8, height=0.6):
    h, w = amap.shape
    y0, x0 = int(h * (1 - height) / 2), int(w * (1 - width) / 2)
    return float(amap[y0:h - y0, x0:w - x0].max())


def save_heatmaps(rows):
    tiles = []
    hi = max(a.max() for _, _, a in rows)
    for path, score, amap in rows:
        img = cv2.resize(cv2.imread(str(path)), (224, 224))
        heat = cv2.resize(amap, (224, 224))
        heat = cv2.applyColorMap((np.clip(heat / hi, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET)
        tile = cv2.addWeighted(img, 0.55, heat, 0.45, 0)
        color = (0, 200, 0) if path.parent.name == "good" else (0, 0, 255)
        cv2.putText(tile, f"{path.parent.name[0]}{path.stem[-2:]} {score:.2f}", (6, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        tiles.append(np.hstack([img, tile]))
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
    cv2.imwrite(str(ROOT / "heatmaps.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])


if __name__ == "__main__":
    main()
