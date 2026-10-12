# Duplo Inspection Cell — Design

**Date:** 2026-10-11
**Status:** draft, awaiting review

## 1. Purpose

A small, complete machine-vision cell built as a portfolio piece for the position
"Инженер Машинно Зрение и AI за Индустриална Автоматизация". It must show, in a 60–90 s video
and a GitHub repo, all four things the job ad asks for:

| Job ad asks for | Where the demo shows it |
|---|---|
| AI for inspection, measurement, recognition | PatchCore anomaly detection + mm measurement + rule checks |
| Handshaking camera ↔ robot/PLC in real time | Modbus RTU handshake between vision PC and Arduino actuator |
| Self-diagnostics, error handling, adaptive decisions | Camera/actuator health checks, fault codes, reject verification |
| Optical setup, calibration, coordinate systems | Locked exposure/WB, depth-based plane + mm calibration, camera-moved detection |

**Success criteria**
- A brick placed on the plate is inspected automatically (no button), result shown within 1 s of the part settling.
- Bad bricks are struck by the stepper "arm" (and the camera confirms they moved); good bricks are left alone.
- On the held-out test session: every defect type in §6 is rejected and no good brick is rejected
  (numbers reported honestly whatever they are).
- Every fault in §8 can be provoked on camera and the cell shows the fault and recovers.
- Built in about a day of the user's time; Python runtime, ONNX model (C++ port possible later).

## 1a. v1 scope — half a day

The whole design below is what the cell *should* do. v1 builds the left column; the right column is
written up (findings doc / README) as "how it should be handled", not built.

| Built in v1 | Documented only |
|---|---|
| Live inspection app with overlay: OK/NG, reason, heatmap, counters | RECHECK band + operator stop |
| Locate (plate box) → align → PatchCore, plus one size rule | Depth measurement in mm, hue rule |
| Modbus RTU handshake with heartbeat and timeouts (§7) | Camera-moved detection, recalibration |
| Arduino firmware: strike, return to rest, report DONE | Limit switch / homing |
| Faults: CAMERA_LOST, ACTUATOR_TIMEOUT / COMM_LOST, REJECT_FAILED | Rest of the fault table (§8), glare/shadow fixes |
| Training script (seeded) + honest results report | Separate test session, C++ port |

**Reject in v1:** the arm strikes the NG brick; it does not have to remove it. VERIFY passes when the
brick moved (center shifted more than half a brick length, pose changed, or no longer located).
Bricks are placed on a target mark in the strike area at the center of the plate box.

## 2. Hardware

- Intel RealSense D435 on USB 3, looking straight down from **35–40 cm** (closer than ~28 cm the
  depth stream fails, as the spike showed).
- Inspection plate: white paper between two fixed toy tubs that cannot be moved. The tubs are kept
  out of the inspection by the hand-fitted plate box (`config/plate_roi.json`); parts must lie fully
  inside it. One lamp only: with both lamps the paper clipped and the hot spot punched holes in the
  depth image (~50 % coverage); with one lamp coverage is 94–99 %. Glare on the studs remains
  (see findings) — a diffuser or polarizer is the planned fix.
- Laptop with RTX 3080 Laptop GPU (training + inference).
- Joy-IT Arduino Mega 2560, a TB6600-type microstep driver (own power supply) and a NEMA 17 on an
  L-bracket. **On this driver's DIP block, switch up = OFF.** Verified setting: switches 1, 2, 5 up,
  3, 4, 6 down = S1 OFF, S2 OFF, S3 ON (1/16, **3200 steps/rev**), S4 ON, S5 OFF, S6 ON (**1.0 A**).
  Confirmed on hardware: 100 steps ≈ 11°, 1600 steps = half a turn. (Before the fix it was on full
  step at 3.0 A — the rose-picker setting read with the wrong switch orientation.)
- Wiring (common cathode, ENA unused so the driver is always enabled and holds the arm):
  PUL+ → D2, DIR+ → D3, PUL− and DIR− → Arduino GND (bridged on the driver). Motor coils on
  A+/A− and B+/B−. The driver's supply GND is never tied to the Arduino. Loose jumper wires caused
  two failed tests — secure them before a demo.
- Reject arm: a glued Г-shaped popsicle-stick frame on a flexible shaft coupler. The motor sits at
  the bottom-left corner beside the tub with the shaft at an angle; at rest the arm lies along the
  table edge with its shadow just outside the plate box, and a reject swings the crossbar over and
  down onto the center of the box.

## 3. Architecture

```
RealSense ──USB3──► Vision app (laptop, Python)
                     camera → diagnostics → locate → align → measure → rules → anomaly → decision
                     cell state machine ── HMI overlay window
                     Modbus RTU master ──USB serial──► Arduino (Modbus RTU slave) → stepper + stick
```

The vision app plays the PLC/master role; the Arduino plays the robot/actuator. No simulated
middle layer: the register map in §7 is what would be configured against a real PLC.

### Code layout

```
cell/
  camera.py       RealSense wrapper: start/reconnect, exposure/WB lock, aligned color+depth, intrinsics
  locate.py       part mask, locate (min-area rect), aligned crop
  measure.py      table-plane fit, pixel→mm via depth deprojection, part length/width/height
  rules.py        tolerance checks → list of NG reasons
  anomaly.py      ONNX Runtime inference, score + heatmap, threshold
  diagnostics.py  frame health: brightness, sharpness, depth validity, camera moved
  actuator.py     Modbus master + handshake; FakeActuator for tests
  cell.py         cell state machine (§5), ties everything together
  hmi.py          overlay drawing
  config.yaml     tolerances, thresholds, serial port, timeouts
app.py            entry point
train/train.py    anomalib PatchCore training on train split, threshold from val split, ONNX export
train/evaluate.py one-shot report on the test split (confusion matrix, per-defect scores, heatmap sheet)
tools/            capture.py, camera_check.py (exist)
firmware/reject_cell/   reject_cell.ino + RejectController (Arduino-free, unit-tested on the PC)
tests/            pytest suite (§11)
```

Each `cell/` module has one job and can be tested without the camera or Arduino:
`locate/measure/rules/anomaly` take arrays and return values; `actuator` hides Modbus behind
`request_reject()` / `step()` (run by `ActuatorThread`); `cell.py` only sees those interfaces.

## 4. Inspection pipeline

1. **Locate** — part mask = pixels clearly above the table plane (depth) OR clearly non-paper
   (color: saturation/darkness). Depth catches low-contrast parts such as the white plate; color
   covers depth holes on dark plastic. Pick the blob nearest the plate center that does not touch
   the image border. Proven in the spike (color only): all 37 good bricks located.
2. **Align** — min-area rectangle → rotate so the long side is horizontal → fixed 320×320 crop.
   The 2×4 brick is 180°-symmetric from above, so no orientation ambiguity.
3. **Measure** — fit the table plane from depth around the part; deproject the rectangle corners
   with the RealSense intrinsics → length/width in mm; height = median height of the part's top
   surface above the plane.
4. **Rules** (geometry — deterministic, explainable):
   - length 63.8 mm ± tol, width 31.8 mm ± tol (2×4 Duplo nominal)
   - height of the stud tops ≈ 23.7 mm ± tol (catches on-side ≈ 31.8 mm, upside-down, stacked parts)
   - mean hue inside the part within a band around the reference brown
   Tolerances set from the train split (mean ± k·σ, with a floor), stored in `config.yaml`.
5. **Anomaly** (appearance — learned): PatchCore on the aligned crop → heatmap. The score is the
   heatmap maximum inside a **part-shaped ROI** (measured part rectangle + margin), not the whole
   crop. Catches surface defects (paper scraps, sticks, prints, marker) and anything the rules miss.

6. **Decision** — three bands, not one threshold:
   - **OK**: all rules pass and score < lower bound.
   - **NG**: any rule fails or score > upper bound. Carries every failing reason code,
     e.g. `NG: WRONG_SIZE, SURFACE_ANOMALY`.
   - **RECHECK**: rules pass, score between the bounds → take a second image and decide again;
     still uncertain → PAUSED for the operator. A borderline part is never silently passed or scrapped.
   - Part not fully inside the plate box → not inspected, `PART_OUTSIDE_ZONE`.

The plate box (`config/plate_roi.json`, fitted in `capture.py`) only decides *where to search*.
Pixels outside it are not painted over (the spike showed the paint seam itself scores as an
anomaly); the part-shaped ROI keeps them out of the score instead.
Spike results and the reasoning: `docs/findings/2026-10-11-anomaly-spike.md`.

Lesson from the spike: 2×2 bricks pass the anomaly model by a hair (0.50–0.53) but fail the size
rule by ~50 %. Geometry belongs to rules; appearance belongs to the model.

## 5. Cell state machine

```
IDLE ──part appears──► SETTLING ──stable N frames──► INSPECTING ──► OK ──part removed──► IDLE
                                                     ▲   │
                                       second image  └───┼──► RECHECK (once; then PAUSED)
                                                         │
                                                         └──► NG ──► REJECTING ──DONE──► VERIFY
VERIFY: part moved (see §1a) → wait for removal → IDLE;  part unmoved → FAULT(REJECT_FAILED)
any state ──operator hand in view──► PAUSED ──view clear──► IDLE
any state ──diagnostic/actuator fault──► FAULT ──X key (reset; R = "stick is home" for NOT_HOME) and condition cleared──► IDLE
```

- **Settling:** part center moves < 2 px over 5 consecutive frames.
- **Hand detection:** a large blob entering from the image border → PAUSED, no inspection. In the reject trigger, a blob over MAX_AREA or one touching the box edge next to a part holds the strike and pauses VERIFY; edge clutter therefore also holds it.
- **Verify** uses vision to confirm the physical reject actually worked — the camera checks the robot.

## 6. Defects

| Code | How it is produced | Caught by |
|---|---|---|
| WRONG_PART | 2×2 brick, other shapes, white plate | size rule (+ depth for the plate) |
| WRONG_POSE | brick on its side, upside down | height rule, anomaly |
| WRONG_COLOR | cyan / other color 2×4 | hue rule, anomaly |
| SURFACE_ANOMALY | dry-erase marker dot, small tape piece, Blu-Tack on a stud, hair | anomaly |
| STACKED | two bricks stuck together | height rule |

All defects are reversible (no bricks are damaged).

## 7. Handshake — Modbus RTU

Laptop = master (pymodbus, 115200 baud, slave id 1). Arduino = slave (ModbusRTUSlave library,
plain USB serial, no RS-485 hardware needed).

| Table | Addr | Name | Direction | Meaning |
|---|---|---|---|---|
| Coil | 0 | REQ_REJECT | master → slave | request one reject cycle |
| Coil | 1 | RESET_FAULT | master → slave | clear actuator fault |
| Discrete input | 0 | READY | slave → master | homed, idle, no fault |
| Discrete input | 1 | BUSY | slave → master | sweep in progress |
| Discrete input | 2 | DONE | slave → master | sweep finished, stick back home |
| Discrete input | 3 | FAULT | slave → master | see FAULT_CODE |
| Holding reg | 0 | MASTER_HEARTBEAT | master → slave | incremented every 100 ms |
| Holding reg | 1 | STRIKE_STEPS | master → slave | signed; negative strikes the other way (default 800) |
| Holding reg | 2 | MAX_SPEED | master → slave | steps/s (default 2400) |
| Holding reg | 3 | ACCELERATION | master → slave | steps/s² (default 12000) |
| Input reg | 0 | SLAVE_HEARTBEAT | slave → master | incremented every 100 ms |
| Input reg | 1 | FAULT_CODE | slave → master | 0 none, 1 comm lost, 2 move timeout, 3 not home |
| Input reg | 2 | CYCLE_COUNT | slave → master | completed rejects |

**Reject sequence (4-way handshake):** master checks READY → sets REQ_REJECT → slave sets BUSY,
sweeps, returns, clears BUSY, sets DONE → master clears REQ_REJECT → slave clears DONE.

**Timeouts (master):** BUSY within 300 ms of REQ; DONE within 2 × the slave's stroke timeout + 0.5 s
(3.1 s at the defaults); slave heartbeat change within 500 ms. **Slave:** each stroke must finish within
1.5 × its ideal time for the set steps, speed and acceleration + 0.5 s (1.3 s at the defaults) →
stop motor, FAULT_CODE=2; master heartbeat unchanged for 1 s → stop motor, FAULT_CODE=1. The master
refuses strike settings outside |steps| 1..32767, speed 1..4000 steps/s, acceleration 1..65535.

**Implementation:** `firmware/reject_cell` (slave; logic in `RejectController`, no Arduino
dependencies) and `cell/actuator.py` (master, non-blocking `step()` run by a background thread
every 50 ms). Details settled while building it:
- Requests are edge-armed: REQ_REJECT must be seen low before a cycle can start, so a held or
  stale request (e.g. raised during a fault) never strikes twice. RESET_FAULT acts on its rising edge.
- After a reset the slave drives the arm home first (READY only once home).
- On connect the master clears REQ_REJECT, writes the strike parameters and auto-resets a
  COMM_LOST fault, retrying until the slave has seen the new heartbeat. Any other slave fault is
  latched for the operator. (Opening the port resets the Mega, so a leftover COMM_LOST is rare.)
- Home is the power-on position, so the firmware keeps an EEPROM flag "away from home", set when a
  stroke leaves home and cleared on arrival. Booting with it set gives FAULT_CODE=3 NOT_HOME: no
  strike until the operator has put the stick home and pressed R (X does not clear it). Only a
  restart while away asks for R; normal start-ups and strikes do not.
- BUSY may be missed when a strike finishes between two polls; DONE is accepted in its place.
  CYCLE_COUNT must rise by exactly one per handshake (ACTUATOR_COUNT_MISMATCH otherwise).
- ModbusRTUSlave holds the Mega's loop for ~4-5 ms per request, during which no steps are taken;
  polling every 50 ms keeps that to a slightly slower strike, with exact positions.
- A full strike at the defaults takes ~1.1-1.3 s on the real stepper (ramps plus the Modbus pauses),
  longer than the 1 s watchdog: a master that dies mid-strike leaves the arm halted part-way.
- A slave COMM_LOST seen while the link is live again (this process stalled > 1 s) is auto-reset the
  same way as at connect. A bug inside the master's step() latches ACTUATOR_INTERNAL instead of
  stopping the thread, and READY needs a status read within the last 500 ms.
- On quit, tools/inspect_test.py stops taking new strikes and waits (up to the DONE timeout) for a
  running cycle to finish before it closes the port. The arm's thread starts only after the camera.

**Homing:** no limit switch — the stick's power-on position is home; the firmware counts steps
and always returns the same number. (A limit switch is a possible later upgrade.)

## 8. Self-diagnostics and faults

| Fault | Detection | Cell reaction |
|---|---|---|
| CAMERA_LOST | frame timeout / device gone | FAULT, auto-reconnect loop, resume when frames return |
| IMAGE_TOO_DARK / TOO_BRIGHT | plate brightness outside band around the locked reference | FAULT, suggest re-lock (L) |
| IMAGE_BLURRED | sharpness below fraction of reference | FAULT |
| CAMERA_MOVED | table plane distance/tilt or paper edges shifted vs calibration | FAULT, recalibrate (C) |
| DEPTH_UNRELIABLE | valid-depth ratio on plate too low | depth rules skipped, flagged on HMI (degraded mode) |
| ACTUATOR_COMM_LOST | Modbus errors / slave heartbeat stale | FAULT, NG parts not rejected, retry connect |
| ACTUATOR_TIMEOUT | BUSY or DONE not seen in time | FAULT, needs reset |
| ACTUATOR NOT_HOME | arm restarted while away from home (EEPROM flag) | FAULT, operator puts the stick home by hand, presses R |
| REJECT_FAILED | VERIFY sees the part still on the plate | FAULT, needs operator |
| OPERATOR_IN_CELL | hand blob from border | PAUSED (not a fault) |

Each fault has a code, a human message on the HMI, and a log line with timestamp.

## 9. HMI overlay

Live image with: part outline + measured mm, OK/NG banner with reason codes, anomaly heatmap
overlay on NG, cell state, handshake lamps (READY/REQ/BUSY/DONE/FAULT), heartbeat indicator,
OK/NG counters, cycle time, last 5 log lines. Keys: L re-lock exposure, C recalibrate,
X reset fault, R stick home (after NOT_HOME), H toggle heatmap, Q quit.

## 10. Datasets, training, evaluation

Photos are captured per session (`capture.py --session sessionN`, good and defect mixed);
`train.py` derives the splits with a fixed seed.

| Split | Source | Content | Use |
|---|---|---|---|
| train | session1 | ~75 % of the good photos (~40) | PatchCore memory bank, rule tolerances |
| val | session1 | remaining good + all defects | choose anomaly threshold |
| test | session2 (later, setup rebuilt) | ~15 good + ~15 defect incl. surface defects | final report, used once |

Training: anomalib PatchCore (wide_resnet50_2, layers 2+3) on aligned crops, export to ONNX,
runtime uses ONNX Runtime (CUDA, CPU fallback). Threshold: midpoint between the highest val-good and
lowest val-defect score; if they overlap, the value that rejects all val defects, and the overlap is
reported. If val has no surface defects (or no good bricks), the side that exists sets the bound and
anomalib's 0.5 fills in for the other; training prints a WARNING and meta.json records
`threshold_basis`. NaN/inf scores stop training. `evaluate.py` prints the confusion matrix and per-defect-type scores and saves a heatmap
sheet; those numbers go into the README unedited.

## 11. Testing

- **Unit (pytest, no hardware):** locate/align on saved spike images (all good located, angle
  normalised); measure on synthetic depth planes; rules table; handshake against a fake slave
  (normal cycle, BUSY timeout, DONE timeout, heartbeat loss); cell state machine with scripted
  frames + FakeActuator (OK path, NG path, REJECT_FAILED, PAUSED).
- **Hardware checks:** `camera_check.py`; a `tools/actuator_check.py` that runs one handshake
  cycle and prints timings.
- **Acceptance:** the test split report (§10) and a filmed run provoking each fault in §8.

## 12. Out of scope

C++ runtime (the ONNX export keeps it possible), real PLC / OPC UA / PROFINET, conveyor or
multiple part types, robot arm / hand-eye calibration, web dashboard.

## 13. Open questions

1. Strike angle (steps from rest to the brick) — measured in the first hardware test.
2. Motor rated current — from the motor label; 1.0 A is the safe starting point.
