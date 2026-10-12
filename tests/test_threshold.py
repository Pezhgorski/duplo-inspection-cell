"""OK/NG threshold selection on the validation scores (cell/threshold.py)."""
import math

import pytest

from cell.threshold import ANOMALIB_THRESHOLD, MARGIN, choose_threshold


def test_separable_takes_the_midpoint():
    assert choose_threshold([0.2, 0.4], [0.6, 0.9]) == (pytest.approx(0.5), "separable")


def test_overlap_still_rejects_every_defect():
    assert choose_threshold([0.3, 0.65], [0.55, 0.9]) == (pytest.approx(0.55 - MARGIN), "overlap")


def test_no_surface_defects_does_not_crash():
    # every defect in val was a wrong size: the size rule caught them, none reach the model
    threshold, basis = choose_threshold([0.2, 0.45], [])
    assert basis == "no surface defects in val"
    assert threshold == ANOMALIB_THRESHOLD  # every good brick still passes


def test_no_surface_defects_with_a_high_good_score():
    threshold, _ = choose_threshold([0.2, 0.7], [])
    assert 0.7 < threshold < 0.71  # never rejects a good brick from val


def test_no_good_bricks_in_val():
    threshold, basis = choose_threshold([], [0.8, 0.9])
    assert basis == "no good bricks in val"
    assert threshold == ANOMALIB_THRESHOLD
    threshold, _ = choose_threshold([], [0.3])
    assert threshold < 0.3  # still rejects every defect


def test_no_scores_at_all():
    assert choose_threshold([], []) == (ANOMALIB_THRESHOLD, "no val scores")


def test_tie_is_an_overlap():
    assert choose_threshold([0.5], [0.5]) == (pytest.approx(0.5 - MARGIN), "overlap")


def test_single_scores_separate():
    assert choose_threshold([0.2], [0.8]) == (pytest.approx(0.5), "separable")


@pytest.mark.parametrize("good, defect", [([0.3], [math.nan, 0.6]), ([math.nan, 0.3], [0.6]),
                                          ([0.3, math.inf], [0.6]), ([], [math.nan])])
def test_non_finite_scores_are_refused(good, defect):
    with pytest.raises(ValueError, match="non-finite"):
        choose_threshold(good, defect)
