"""Live inspection test: inspects the brick on the plate and strikes NG parts with the arm.

Usage:  python tools/inspect_test.py [--port COM4] [--strike-steps 800] [--reverse] [--no-motor]

Keys:
  T      real-time mode on/off: inspect every frame (starts on)
  A      auto-reject on/off: in real-time mode a settled NG part is struck once, then the
         camera verifies it moved (starts on when the arm is connected)
  SPACE  inspect once and freeze the result (turns real-time off)
  K      strike now (manual), through the same Modbus handshake
  R      the stick is back home: only asked for after the arm restarted mid-strike (NOT_HOME)
  X      reset an arm fault
  L      re-lock camera exposure / white balance
  Q/ESC  quit

Needs models/meta.json from train/train_model.py, and the reject_cell firmware on the Arduino
(Modbus RTU slave, see cell/actuator.py). The driver is set to 3200 steps per turn (1/16):
800 steps = a quarter turn.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from capture import (AUTO_SETTLE_FRAMES, CameraSettings, color_sensor,  # noqa: E402
                     reset_depth_sensor, start_pipeline)
from cell.actuator import (COIL_REQ_REJECT, ActuatorMaster, ActuatorThread, BusError, ModbusBus,  # noqa: E402
                           check_strike)
from cell.anomaly import AnomalyModel, roi_score  # noqa: E402
from cell.geometry import aligned_crop, find_part, load_plate, part_box, plate_pixels  # noqa: E402
from cell.reject import RejectTrigger  # noqa: E402
from cell.rules import check_size  # noqa: E402

WINDOW = "inspect test"
PANEL = 480
GREEN, RED, YELLOW, WHITE = (0, 200, 0), (0, 0, 230), (0, 220, 255), (255, 255, 255)
SEARCH_MESSAGES = {
    "empty": "NO PART in the plate box",
    "outside": "PART NOT FULLY INSIDE the plate box",
    "multiple": "MORE THAN ONE PART in the plate box",
    "too_large": "SOMETHING TOO BIG in the plate box (hand?)",
}


def inspect(frame, plate, model, meta) -> dict:
    search = find_part(frame, plate)
    if search.status != "found":
        return {"verdict": "NO INSPECTION", "color": YELLOW, "status": search.status,
                "crowded": search.crowded, "lines": [SEARCH_MESSAGES[search.status]]}
    part = search.part
    crop = aligned_crop(frame, part)
    start = time.time()
    heat = model.heatmap(crop)
    ms = (time.time() - start) * 1000
    score = roi_score(heat, part_box(part))
    reasons = check_size(part, meta["size_reference"])
    if score > meta["threshold"]:
        reasons.append("SURFACE_ANOMALY")
    ok = not reasons
    return {
        "verdict": "OK" if ok else "NG", "color": GREEN if ok else RED, "crop": crop, "heat": heat,
        "box": part_box(part), "part": part, "status": search.status, "crowded": search.crowded,
        "lines": [", ".join(reasons) if reasons else "all checks passed",
                  f"anomaly score {score:.3f}  (threshold {meta['threshold']:.3f})",
                  f"size {part.length:.0f} x {part.width:.0f} px",
                  f"inference {ms:.0f} ms"],
    }


def result_panel(result) -> np.ndarray:
    panel = np.full((720, PANEL, 3), 30, np.uint8)
    if result is None:
        cv2.putText(panel, "press SPACE to inspect", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, WHITE, 2, cv2.LINE_AA)
        return panel
    if "crop" in result:
        crop = cv2.resize(result["crop"], (PANEL, PANEL))
        heat = cv2.applyColorMap((np.clip(cv2.resize(result["heat"], (PANEL, PANEL)), 0, 1) * 255)
                                 .astype(np.uint8), cv2.COLORMAP_JET)
        view = cv2.addWeighted(crop, 0.55, heat, 0.45, 0)
        s = PANEL / result["crop"].shape[0]
        x0, y0, x1, y1 = (int(v * s) for v in result["box"])
        cv2.rectangle(view, (x0, y0), (x1, y1), WHITE, 1)
        panel[:PANEL] = view
    y = PANEL + 55
    cv2.putText(panel, result["verdict"], (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.6, result["color"], 4, cv2.LINE_AA)
    for line in result["lines"]:
        y += 36
        cv2.putText(panel, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
    return panel


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--strike-steps", type=int, default=800, help="800 = quarter turn at 3200 steps/rev")
    parser.add_argument("--speed", type=int, default=2400, help="steps/s")
    parser.add_argument("--accel", type=int, default=12000, help="steps/s^2")
    parser.add_argument("--reverse", action="store_true", help="strike in the other direction")
    parser.add_argument("--no-motor", action="store_true", help="run without the Arduino")
    args = parser.parse_args()

    meta = json.loads((ROOT / "models" / "meta.json").read_text())
    print("loading model...")
    model = AnomalyModel(ROOT / meta["model"])
    plate = load_plate()
    actuator = None
    if not args.no_motor:
        steps = -args.strike_steps if args.reverse else args.strike_steps
        try:
            check_strike(steps, args.speed, args.accel)
        except ValueError as exc:
            parser.error(str(exc))
        master = ActuatorMaster(ModbusBus.serial(args.port), strike_steps=steps, speed=args.speed, accel=args.accel)
        actuator = ActuatorThread(master).start()
    trigger, auto_reject = RejectTrigger(), actuator is not None

    pipeline, profile = start_pipeline()
    reset_depth_sensor(profile)
    settings = CameraSettings(color_sensor(profile))
    settings.set_auto()
    grab = lambda: np.asanyarray(pipeline.wait_for_frames().get_color_frame().get_data())
    relock_at, frames_seen = AUTO_SETTLE_FRAMES, 0
    result, status = None, "metering camera..."
    realtime, fps, last_tick = True, 0.0, time.time()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = grab().copy()
            frames_seen += 1
            if relock_at is not None and frames_seen >= relock_at:
                settings.meter_and_lock(grab)
                relock_at, status = None, "camera locked"
                continue
            if realtime and relock_at is None:
                result = inspect(frame, plate, model, meta)
                if auto_reject:
                    trigger.update(result["status"], result["verdict"], result.get("part"), actuator,
                                   crowded=result["crowded"])
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - last_tick, 1e-3)  # smoothed loop rate
            last_tick = now

            view = frame.copy()
            x0, y0, x1, y1 = plate_pixels(plate, view.shape[1], view.shape[0])
            cv2.rectangle(view, (x0, y0), (x1, y1), WHITE, 1)
            if result and "part" in result:
                corners = cv2.boxPoints((result["part"].center,
                                         (result["part"].length, result["part"].width), result["part"].angle))
                cv2.polylines(view, [corners.astype(np.int32)], True, result["color"], 2)
            mode = f"REAL-TIME {fps:.0f} fps" if realtime else "SNAPSHOT"
            cv2.putText(view, f"{mode} | {status}", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2, cv2.LINE_AA)
            arm_lines = ["arm: disabled (--no-motor)"] if actuator is None else [
                actuator.describe(),
                f"auto-reject {'ON' if auto_reject else 'OFF'}" + (f" | {trigger.message}" if trigger.message else "")]
            for i, line in enumerate(arm_lines):
                bad = actuator is not None and i == 0 and actuator.master.fault
                cv2.putText(view, line, (12, 60 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            RED if bad else WHITE, 2 if bad else 1, cv2.LINE_AA)
            cv2.putText(view, "T real-time | A auto | SPACE snapshot | K strike | R stick home | X reset | L re-lock | Q quit",
                        (12, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
            cv2.imshow(WINDOW, np.hstack([view, result_panel(result)]))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("t"):
                realtime = not realtime
            elif key == ord(" ") and relock_at is None:
                realtime = False
                result = inspect(frame, plate, model, meta)
                status = f"inspected: {result['verdict']}"
            elif key == ord("a") and actuator is not None:
                auto_reject = not auto_reject
                trigger = RejectTrigger()
            elif key == ord("k"):
                if actuator is None:
                    status = "motor disabled (--no-motor)"
                else:
                    status = "strike requested" if actuator.request_reject() else "arm not ready"
            elif key == ord("x") and actuator is not None:
                actuator.reset()
                status = "arm reset requested"
            elif key == ord("r") and actuator is not None:
                actuator.confirm_home()
                status = "stick home confirmed"
            elif key == ord("l"):
                settings.set_auto()
                relock_at, status = frames_seen + AUTO_SETTLE_FRAMES, "re-metering camera..."
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if actuator:
            actuator.stop()
            try:
                actuator.master.bus.write_coil(COIL_REQ_REJECT, False)  # leave no request behind
            except BusError:
                pass
            actuator.master.bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
