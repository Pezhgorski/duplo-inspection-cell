"""Live RealSense capture tool for building the datasets.

Usage:  capture.py --session session1

Keys:
  SPACE  save frame as GOOD          D  save frame as DEFECT
  U      undo last save              L  re-run auto exposure / white balance, then lock it
  [ / ]  exposure down / up          Q / ESC  quit
  - / +  shrink / grow the plate box  arrows  move the plate box  (saved to config/plate_roi.json)

Color images go to data/<session>/<good|defect>/NNNN.png (what the anomaly model sees),
the aligned 16-bit depth to data/<session>/depth/<good|defect>/NNNN.png (for the depth checks).
The training script splits a session into train/val; a later session is the test set.
The plate box is the inspection ROI: parts are only searched inside it, everything outside is ignored.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"  # replaced by data/<session> in main()
CLASSES = ("good", "defect")
WINDOW = "capture"
AUTO_SETTLE_FRAMES = 45
EXPOSURE_STEP = 1.25  # multiplicative step for [ and ]
DEPTH_MIN_VALID = 80  # % of the placement box with depth; below this the depth checks are unreliable
LOCK_TOLERANCE = 0.15  # locked brightness may differ this much from the auto-exposure image
ROI_FILE = ROOT / "config" / "plate_roi.json"
ROI_DEFAULT = {"cx": 0.5, "cy": 0.5, "w": 0.35, "h": 0.45}  # fractions of image width/height
ROI_STEP = 0.95  # size factor per - press
ROI_MOVE = 0.01  # fraction of the image per arrow press
# cv2.waitKeyEx codes for the arrow keys on Windows
ARROWS = {2424832: (-1, 0), 2555904: (1, 0), 2490368: (0, -1), 2621440: (0, 1)}


def load_roi():
    try:
        return {**ROI_DEFAULT, **json.loads(ROI_FILE.read_text())}
    except (FileNotFoundError, ValueError):
        return dict(ROI_DEFAULT)


def save_roi(roi):
    ROI_FILE.parent.mkdir(exist_ok=True)
    ROI_FILE.write_text(json.dumps(roi, indent=2))


def scale_roi(roi, factor):
    w, h = roi["w"] * factor, roi["h"] * factor
    if max(w, h) > 1.0 or min(w, h) < 0.05:
        return roi
    return {**roi, "w": round(w, 4), "h": round(h, 4)}


def move_roi(roi, dx, dy):
    cx = min(max(roi["cx"] + dx * ROI_MOVE, roi["w"] / 2), 1 - roi["w"] / 2)
    cy = min(max(roi["cy"] + dy * ROI_MOVE, roi["h"] / 2), 1 - roi["h"] / 2)
    return {**roi, "cx": round(cx, 4), "cy": round(cy, 4)}


def roi_pixels(roi, width, height):
    """(x0, y0, x1, y1) of the plate box in pixels."""
    bw, bh = int(width * roi["w"]), int(height * roi["h"])
    x0, y0 = int(width * roi["cx"] - bw / 2), int(height * roi["cy"] - bh / 2)
    return max(x0, 0), max(y0, 0), min(x0 + bw, width), min(y0 + bh, height)


def start_pipeline():
    pipeline = rs.pipeline()
    for w, h, fps in ((1280, 720, 30), (640, 480, 30), (640, 480, 15)):
        config = rs.config()
        config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        try:
            profile = pipeline.start(config)
            print(f"streaming {w}x{h} @ {fps} fps")
            return pipeline, profile
        except RuntimeError:
            continue
    raise RuntimeError("Could not start a color+depth stream. Is the camera plugged in?")


def reset_depth_sensor(profile):
    """Depth options persist on the device across sessions, so always start from a known state."""
    sensor = profile.get_device().first_depth_sensor()
    sensor.set_option(rs.option.visual_preset, float(rs.rs400_visual_preset.default))
    sensor.set_option(rs.option.emitter_enabled, 1)
    sensor.set_option(rs.option.laser_power, sensor.get_option_range(rs.option.laser_power).default)


def color_sensor(profile):
    for sensor in profile.get_device().query_sensors():
        if sensor.get_info(rs.camera_info.name) == "RGB Camera":
            return sensor
    raise RuntimeError("RGB sensor not found")


class CameraSettings:
    """Locks exposure / white balance so the image does not drift between shots.

    The D435 RGB sensor does not report the values its auto mode picked (and frame
    metadata is unavailable without the Windows metadata registry patch), so locking
    works by measuring the auto image and searching manual settings that reproduce it.
    """

    def __init__(self, sensor):
        self.sensor = sensor
        self.locked = False

    def set_auto(self):
        self.sensor.set_option(rs.option.enable_auto_exposure, 1)
        self.sensor.set_option(rs.option.enable_auto_white_balance, 1)
        self.locked = False

    def meter_and_lock(self, grab):
        """grab() returns the latest BGR color image. Takes a few seconds."""
        reference = self._measure(grab)
        target_brightness = reference.mean()
        target_ratios = self._ratios(reference)

        self.sensor.set_option(rs.option.enable_auto_exposure, 0)
        self.sensor.set_option(rs.option.enable_auto_white_balance, 0)

        # exposure first: color ratios measured on a near-black image are just noise.
        # Exposure is quantized coarsely, so match it first and fine-tune with gain at the end
        self._set(rs.option.gain, self.sensor.get_option_range(rs.option.gain).default)
        self._set(rs.option.exposure, self.sensor.get_option_range(rs.option.exposure).default)
        for _ in range(15):  # frames already in flight still carry the auto settings
            grab()
        self._converge(rs.option.exposure, grab, target_brightness, tries=6)

        # the color response is steep (2800K -> 3700K moves R/G from 1.0 to 1.5), so coarse then fine
        rng = self.sensor.get_option_range(rs.option.white_balance)
        coarse = self._best_white_balance(grab, target_ratios, np.arange(rng.min, rng.max + 1, 300))
        fine = np.arange(max(rng.min, coarse - 300), min(rng.max, coarse + 300) + 1, 50)
        self._set(rs.option.white_balance, self._best_white_balance(grab, target_ratios, fine))
        self._converge(rs.option.gain, grab, target_brightness, tries=4)
        self.locked = True
        # how close the locked image is to what auto mode produced (1.0 = identical brightness)
        return self._measure(grab).mean() / max(target_brightness, 1.0)

    def scale_exposure(self, factor):
        if not self.locked:
            return
        # step until the quantized value really changes
        current = self.sensor.get_option(rs.option.exposure)
        value = current
        for _ in range(8):
            value *= factor
            self._set(rs.option.exposure, value)
            if self.sensor.get_option(rs.option.exposure) != current:
                return

    def describe(self):
        if not self.locked:
            return "AUTO (press L to lock)"
        exp = self.sensor.get_option(rs.option.exposure)
        wb = self.sensor.get_option(rs.option.white_balance)
        gain = self.sensor.get_option(rs.option.gain)
        return f"LOCKED exp {exp:.0f}  wb {wb:.0f}K  gain {gain:.0f}"

    def _best_white_balance(self, grab, target_ratios, candidates):
        errors = []
        for wb in candidates:
            self._set(rs.option.white_balance, wb)
            ratios = self._ratios(self._measure(grab, skip=3, average=2))
            errors.append(np.abs(ratios - target_ratios).sum())
        return candidates[int(np.argmin(errors))]

    def _converge(self, option, grab, target, tries):
        # brightness is roughly proportional to exposure and gain
        for _ in range(tries):
            brightness = self._measure(grab).mean()
            if abs(brightness - target) < 2:
                return
            self._set(option, self.sensor.get_option(option) * target / max(brightness, 1.0))

    @staticmethod
    def _measure(grab, skip=6, average=3):
        # settings take a few frames to apply
        for _ in range(skip):
            grab()
        return np.mean([grab().astype(np.float32) for _ in range(average)], axis=0)

    @staticmethod
    def _ratios(image):
        b, g, r = image.reshape(-1, 3).mean(axis=0)
        return np.array([r / max(g, 1.0), b / max(g, 1.0)])

    def _set(self, option, value):
        rng = self.sensor.get_option_range(option)
        value = min(max(value, rng.min), rng.max)
        if rng.step > 0:
            value = rng.min + round((value - rng.min) / rng.step) * rng.step
        self.sensor.set_option(option, value)


def next_index():
    existing = [int(p.stem) for c in CLASSES for p in (DATA / c).glob("*.png") if p.stem.isdigit()]
    return max(existing, default=0) + 1


def count(cls):
    return len(list((DATA / cls).glob("*.png")))


def save(cls, idx, color, depth):
    color_path = DATA / cls / f"{idx:04d}.png"
    depth_path = DATA / "depth" / cls / f"{idx:04d}.png"
    cv2.imwrite(str(color_path), color)
    cv2.imwrite(str(depth_path), depth)  # uint16, millimetre-scale units of the sensor
    return color_path, depth_path


def sharpness(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def draw_hud(view, settings, last_msg, flash_color, flash_until, depth=None, plate=ROI_DEFAULT):
    h, w = view.shape[:2]
    gray = cv2.cvtColor(view, cv2.COLOR_BGR2GRAY)
    # plate box = inspection ROI: dim everything outside it, since the inspection ignores it
    x0, y0, x1, y1 = roi_pixels(plate, w, h)
    outside = np.ones((h, w), bool)
    outside[y0:y1, x0:x1] = False
    view[outside] = (view[outside] * 0.4).astype(np.uint8)
    cv2.rectangle(view, (x0, y0), (x1, y1), (255, 255, 255), 1)
    cv2.drawMarker(view, ((x0 + x1) // 2, (y0 + y1) // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

    roi = gray[y0:y1, x0:x1]
    image_line = f"brightness {roi.mean():.0f}   sharpness {sharpness(roi):.0f}"
    if depth is not None:
        valid = np.mean(depth[y0:y1, x0:x1] > 0) * 100
        image_line += f"   depth valid {valid:.0f}%" + ("  << LOW, check lighting" if valid < DEPTH_MIN_VALID else "")
    lines = [
        f"[{DATA.name}]  good: {count('good')}   defect: {count('defect')}",
        f"camera: {settings.describe()}",
        image_line,
        f"plate box {plate['w']:.0%} x {plate['h']:.0%}  (- / + resize, arrows move)",
        "SPACE good | D defect | U undo | L lock | [ ] exposure | Q quit",
    ]
    if last_msg:
        lines.append(last_msg)
    panel_w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0][0] for t in lines) + 24
    panel = view[0:12 + len(lines) * 26, 0:min(panel_w, w)]
    panel[:] = (panel * 0.35).astype(np.uint8)
    for i, text in enumerate(lines):
        cv2.putText(view, text, (12, 28 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    if time.time() < flash_until:
        cv2.rectangle(view, (0, 0), (w - 1, h - 1), flash_color, 12)


def main() -> int:
    global DATA
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", required=True, help="folder name under data/, e.g. session1")
    DATA = ROOT / "data" / parser.parse_args().session

    for cls in CLASSES:
        (DATA / cls).mkdir(parents=True, exist_ok=True)
        (DATA / "depth" / cls).mkdir(parents=True, exist_ok=True)

    pipeline, profile = start_pipeline()
    align = rs.align(rs.stream.color)
    reset_depth_sensor(profile)
    settings = CameraSettings(color_sensor(profile))
    settings.set_auto()

    plate = load_roi()
    frames_seen = 0
    relock_at = AUTO_SETTLE_FRAMES  # lock automatically once auto exposure has settled
    history = []  # saved file pairs, for undo
    last_msg, flash_color, flash_until = "", (0, 0, 0), 0.0

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            frames_seen += 1
            if relock_at is not None and frames_seen >= relock_at:
                busy = np.asanyarray(color_frame.get_data()).copy()
                draw_hud(busy, settings, "metering exposure / white balance...", (0, 0, 0), 0, plate=plate)
                cv2.imshow(WINDOW, busy)
                cv2.waitKey(1)
                match = settings.meter_and_lock(
                    lambda: np.asanyarray(pipeline.wait_for_frames().get_color_frame().get_data()))
                relock_at = None
                if abs(match - 1.0) > LOCK_TOLERANCE:
                    last_msg = f"LOCK MISMATCH: image is {match:.0%} of auto brightness - press L"
                else:
                    last_msg = f"camera locked ({match:.0%} of auto brightness)"
                continue

            color = np.asanyarray(color_frame.get_data()).copy()
            depth = np.asanyarray(depth_frame.get_data()).copy()

            view = color.copy()
            draw_hud(view, settings, last_msg, flash_color, flash_until, depth, plate)
            cv2.imshow(WINDOW, view)

            full_key = cv2.waitKeyEx(1)
            if full_key in ARROWS:
                plate = move_roi(plate, *ARROWS[full_key])
                save_roi(plate)
                continue
            key = full_key & 0xFF if full_key != -1 else 255
            if key in (ord("q"), 27):
                break
            if key in (ord(" "), ord("d")):
                cls = "good" if key == ord(" ") else "defect"
                if not settings.locked:
                    last_msg = "not saved: camera settings not locked yet"
                    continue
                paths = save(cls, next_index(), color, depth)
                history.append(paths)
                last_msg = f"saved {cls}/{paths[0].name}"
                flash_color = (0, 200, 0) if cls == "good" else (0, 0, 230)
                flash_until = time.time() + 0.25
            elif key == ord("u") and history:
                for p in history.pop():
                    p.unlink(missing_ok=True)
                last_msg = "undone"
            elif key == ord("l"):
                settings.set_auto()
                relock_at = frames_seen + AUTO_SETTLE_FRAMES
                last_msg = "re-metering..."
            elif key == ord("["):
                settings.scale_exposure(1 / EXPOSURE_STEP)
            elif key == ord("]"):
                settings.scale_exposure(EXPOSURE_STEP)
            elif key in (ord("-"), ord("+"), ord("=")):  # = is + without shift
                plate = scale_roi(plate, ROI_STEP if key == ord("-") else 1 / ROI_STEP)
                save_roi(plate)
                last_msg = f"plate box {plate['w']:.0%} x {plate['h']:.0%} saved"
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
