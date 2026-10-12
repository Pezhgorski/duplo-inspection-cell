"""OK/NG threshold on the anomaly score, chosen on the validation split by train/train_model.py."""

ANOMALIB_THRESHOLD = 0.5  # the normalized anomaly maps put anomalib's own threshold at 0.5
MARGIN = 1e-4


def choose_threshold(good: list[float], defect: list[float]) -> tuple[float, str]:
    """Returns (threshold, basis). A score above the threshold is NG.

    - "separable": midpoint between the highest good and the lowest defect score;
    - "overlap": just below the lowest defect, so every defect is still rejected;
    - one side missing (e.g. every defect was a size defect, or too few good bricks to fill val):
      the side that exists sets the bound, anomalib's 0.5 fills in for the other.
    """
    if good and defect:
        if max(good) < min(defect):
            return (max(good) + min(defect)) / 2, "separable"
        return min(defect) - MARGIN, "overlap"
    if good:
        return max(max(good) + MARGIN, ANOMALIB_THRESHOLD), "no surface defects in val"
    if defect:
        return min(min(defect) - MARGIN, ANOMALIB_THRESHOLD), "no good bricks in val"
    return ANOMALIB_THRESHOLD, "no val scores"
