"""Train PatchCore on one capture session and write models/meta.json for the live inspection.

Split (seeded): every 4th located good brick -> val, the rest -> train; all defects -> val.
The model learns from good bricks only. The OK/NG threshold is chosen on val:
- midpoint between the highest good score and the lowest defect score, if they separate;
- otherwise the value that still rejects every defect, and the overlap is reported.
Only defects that pass the size rule count for the threshold - wrong shapes are the rule's job.

Usage:  python train/train_model.py [--session session1]
"""
import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from anomalib.data import Folder  # noqa: E402
from anomalib.engine import Engine  # noqa: E402
from anomalib.models import Patchcore  # noqa: E402
from lightning.pytorch import seed_everything  # noqa: E402

from cell.anomaly import AnomalyModel, roi_score  # noqa: E402
from cell.geometry import CROP_SIZE, aligned_crop, find_part, load_plate, part_box  # noqa: E402
from cell.rules import check_size, size_reference  # noqa: E402

MODELS = ROOT / "models"
CROPS = MODELS / "crops"
SEED = 42


def build_crops(session: Path, plate: dict) -> dict:
    """Locate, align and crop every photo; returns {split/label: [(name, Part)]}."""
    shutil.rmtree(CROPS, ignore_errors=True)
    splits = {"train/good": [], "val/good": [], "val/defect": []}
    for d in splits:
        (CROPS / d).mkdir(parents=True)
    skipped = []
    for label in ("good", "defect"):
        located = []
        for path in sorted((session / label).glob("*.png")):
            img = cv2.imread(str(path))
            search = find_part(img, plate)
            if search.status != "found":
                skipped.append(f"{label}/{path.stem} ({search.status})")
                continue
            located.append((path.name, img, search.part))
        for i, (name, img, part) in enumerate(located):
            split = ("val/good" if i % 4 == 3 else "train/good") if label == "good" else "val/defect"
            cv2.imwrite(str(CROPS / split / name), aligned_crop(img, part))
            splits[split].append((name, part))
    print("skipped (not a single part fully inside the plate box):", skipped or "none")
    for d, items in splits.items():
        print(f"{d}: {len(items)}")
    return splits


def train() -> Path:
    datamodule = Folder(
        name="duplo", root=CROPS,
        normal_dir="train/good", normal_test_dir="val/good", abnormal_dir="val/defect",
        val_split_mode="same_as_test", train_batch_size=8, eval_batch_size=8, num_workers=0,
    )
    model = Patchcore(coreset_sampling_ratio=0.25)
    engine = Engine(default_root_dir=MODELS / "runs", logger=False)
    start = time.time()
    engine.fit(model=model, datamodule=datamodule)
    print(f"training took {time.time() - start:.1f}s")
    return model.to_torch(MODELS)


def choose_threshold(good: list[float], defect: list[float]) -> tuple[float, bool]:
    """Returns (threshold, separable)."""
    if max(good) < min(defect):
        return (max(good) + min(defect)) / 2, True
    return min(defect) - 1e-4, False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", default="session1")
    session = ROOT / "data" / parser.parse_args().session
    seed_everything(SEED)  # PatchCore's coreset sampling is random

    plate = load_plate()
    splits = build_crops(session, plate)
    model_path = train()

    ref = size_reference([part for _, part in splits["train/good"]])
    anomaly = AnomalyModel(model_path)
    results = []
    for split in ("val/good", "val/defect"):
        for name, part in splits[split]:
            heat = anomaly.heatmap(cv2.imread(str(CROPS / split / name)))
            results.append({"name": name, "label": split.split("/")[1],
                             "score": roi_score(heat, part_box(part)), "size": check_size(part, ref)})

    good = [r["score"] for r in results if r["label"] == "good"]
    surface = [r["score"] for r in results if r["label"] == "defect" and not r["size"]]
    threshold, separable = choose_threshold(good, surface)

    for r in sorted(results, key=lambda r: r["score"]):
        verdict = "NG" if r["size"] or r["score"] > threshold else "OK"
        r["verdict"] = verdict
        mark = "" if (verdict == "OK") == (r["label"] == "good") else "   <-- wrong"
        print(f"{r['score']:.3f}  {r['label']:6} {r['name']}  {verdict} {' '.join(r['size'])}{mark}")
    false_rejects = sum(r["label"] == "good" and r["verdict"] == "NG" for r in results)
    missed = sum(r["label"] == "defect" and r["verdict"] == "OK" for r in results)
    print(f"\nthreshold {threshold:.3f} ({'separable' if separable else 'OVERLAP: set to reject every defect'})")
    print(f"val: {len(good)} good, {len(results) - len(good)} defect -> "
          f"{false_rejects} good falsely rejected, {missed} defects missed")

    meta = {
        "model": str(model_path.relative_to(ROOT)), "session": session.name, "seed": SEED,
        "crop_size": CROP_SIZE, "threshold": threshold, "separable": separable,
        "size_reference": ref, "val": results,
    }
    (MODELS / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"wrote {MODELS / 'meta.json'}")


if __name__ == "__main__":
    main()
