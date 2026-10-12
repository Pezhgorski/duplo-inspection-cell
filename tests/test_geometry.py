"""Part search in the plate box on synthetic images (cell/geometry.py)."""
import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from cell.geometry import MAX_AREA, aligned_crop, find_part, plate_pixels  # noqa: E402

PLATE = {"cx": 0.5, "cy": 0.5, "w": 0.4, "h": 0.6}  # 512 x 432 px box in a 1280 x 720 image
RED = (0, 0, 220)


def image(*rects, color=RED):
    """White paper with filled rotated rectangles ((cx, cy), (w, h), angle)."""
    img = np.full((720, 1280, 3), 255, np.uint8)
    for rect in rects:
        cv2.fillPoly(img, [cv2.boxPoints(rect).astype(np.int32)], color)
    return img


def test_empty_box():
    assert find_part(image(), PLATE).status == "empty"


def test_single_brick_is_found_and_measured():
    search = find_part(image(((640, 360), (170, 85), 30)), PLATE)
    assert search.status == "found"
    assert search.part.length == pytest.approx(170, abs=3)
    assert search.part.width == pytest.approx(85, abs=3)
    assert search.part.center == pytest.approx((640, 360), abs=2)


@pytest.mark.parametrize("angle", [0, 20, 45, 70, 90, 110, 135, 160, -30])
def test_aligned_crop_lays_the_long_side_horizontal(angle):
    search = find_part(image(((640, 360), (170, 85), angle)), PLATE)
    crop = aligned_crop(image(((640, 360), (170, 85), angle)), search.part)
    ys, xs = np.nonzero(crop[..., 1] < 100)
    assert np.ptp(xs) + 1 == pytest.approx(170, abs=4)
    assert np.ptp(ys) + 1 == pytest.approx(85, abs=4)


def test_two_bricks_are_multiple():
    assert find_part(image(((520, 360), (120, 60), 0), ((760, 360), (120, 60), 0)), PLATE).status == "multiple"


def test_brick_across_the_box_edge_is_outside():
    x0, _, _, _ = plate_pixels(PLATE, 1280, 720)
    assert find_part(image(((x0, 360), (170, 85), 0)), PLATE).status == "outside"


def test_hand_inside_the_box_is_too_large_not_outside():
    # a blob bigger than any part, fully inside the box: before, this read "PART NOT FULLY INSIDE"
    hand = ((640, 360), (300, 260), 10)
    assert hand[1][0] * hand[1][1] > MAX_AREA
    assert find_part(image(hand), PLATE).status == "too_large"


def test_hand_reaching_in_from_the_edge_is_too_large():
    x0, _, _, _ = plate_pixels(PLATE, 1280, 720)
    assert find_part(image(((x0 + 100, 360), (400, 300), 0)), PLATE).status == "too_large"  # 300 x 300 inside


def test_a_complete_brick_still_wins_over_edge_clutter():
    x0, _, _, _ = plate_pixels(PLATE, 1280, 720)
    search = find_part(image(((640, 360), (170, 85), 0), ((x0, 200), (60, 60), 0)), PLATE)
    assert search.status == "found"


def test_hand_next_to_a_complete_brick_marks_the_search_crowded():
    brick, hand = ((480, 360), (170, 85), 0), ((740, 360), (260, 240), 0)
    assert hand[1][0] * hand[1][1] > MAX_AREA
    search = find_part(image(brick, hand), PLATE)
    assert search.status == "found"   # still inspected: training data and the HMI are unchanged
    assert search.crowded             # but the trigger must not strike
    assert search.part.center == pytest.approx((480, 360), abs=2)


def test_single_brick_is_not_crowded():
    assert not find_part(image(((640, 360), (170, 85), 30)), PLATE).crowded
