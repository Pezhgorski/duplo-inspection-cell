# Review Fixes for the Reject Cell — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the Important and Minor findings from the code reviews of PR #1 (Modbus reject handshake) and PR #2 (threshold / `too_large`), both merged into `main` on 2026-10-12, before the first powered test of the arm.

**Architecture:** The reject cell has three layers. The changes stay inside them:
- the slave's logic, `firmware/reject_cell/RejectController` (Arduino-free C++, unit-tested on the PC);
- the Modbus master, `cell/actuator.py` (a non-blocking `step()` run by `ActuatorThread`);
- the camera-side strike trigger, `cell/reject.py`, fed by `cell/geometry.find_part`.

`tools/inspect_test.py` ties them together. No new modules and no new dependencies.

**Tech Stack:** Python 3.12 (`.venv`), pytest, OpenCV, pymodbus 3.x. C++11 for the AVR (Arduino Mega 2560, AccelStepper, ModbusRTUSlave 3.x, EEPROM). On the PC the C++ builds with g++/clang++ or MSVC 2022 Build Tools.

**Spec:** `docs/superpowers/specs/2026-10-11-duplo-inspection-design.md` (§5 cell state machine, §7 handshake, §8 faults, §10 training). The review findings this plan answers are summarised under each task.

## Global Constraints

- The register map in `firmware/reject_cell/RejectController.h` and `cell/actuator.py` must stay identical: same addresses and same fault codes.
- C++ must build warning-free with `g++ -std=c++11 -Wall -Wextra -Werror` and `cl /std:c++14 /W4 /WX`. On the Mega `float` is 32-bit and RAM is 8 KB, so no heap and no `std::` containers in firmware code.
- Use explicit casts for long → float and float → integer in the firmware. MSVC `/W4` warns on implicit ones (C4244).
- Run tests with `.venv\Scripts\python -m pytest` (the system Python has no pytest). Before this plan: 61 passed and 7 skipped. After Task 1: 68 passed.
- `README.md` and `docs/demo_bg.md` are in Bulgarian, so keep README edits in Bulgarian. The spec and code comments are in English.
- No new pip or Arduino library dependencies. `EEPROM.h` is part of the AVR core.
- Every commit message ends with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

These five inputs are the most likely to hurt someone at the cell. Each is pinned by a test in the task named:

1. **An operator's hand in the plate box next to an NG brick:** no strike until the hand has gone (spec §5, "hand in view → PAUSED"). Covered by Task 2: `test_hand_next_to_an_ng_part_holds_the_strike`, `test_hand_next_to_a_complete_brick_marks_the_search_crowded`.
2. **The first powered test at a slow strike speed (300–600 steps/s):** the cycle completes, with no `MOVE_TIMEOUT` and no `ACTUATOR_TIMEOUT`. Covered by Task 3: `test_slow_strike_is_not_a_move_timeout` (C++) and `test_slow_strike_is_not_a_timeout` (Python).
3. **The Mega restarts while the stick is out** (app crash mid-strike, USB brown-out, or pymodbus reopening the port): the arm reports `NOT_HOME` and waits until the operator puts the stick home and presses R. It never silently takes the current position as the new home, and a restart with the stick at home asks for nothing. Covered by Task 4: `test_boot_away_from_home_faults_until_reset` and `test_reboot_away_from_home_is_latched_for_the_operator`.
4. **Q pressed during a strike:** the stick finishes the cycle and is home before the port closes. Covered by Task 6: `test_stop_lets_a_running_strike_finish`.
5. **An unexpected exception inside the actuator thread:** the job fails, the HMI shows a fault, and the trigger leaves STRIKING. Covered by Task 5: `test_thread_survives_a_bug_in_step_and_reports_it`.

**Deliberately not in this plan.** Each is a separate piece of work:
- The full PAUSED / OPERATOR_IN_CELL state of spec §5 and §8. The spec puts it in "documented only" for v1.
- A `FakeArm` with acceleration ramps.
- Opening the serial port without toggling DTR, so the Mega is not reset on connect.
- The RECHECK band.

---

### Task 1: Build the firmware tests with MSVC when there is no g++

**Why:** `tests/test_firmware.py` only looks for g++/clang++ on PATH, so on this machine all 7 firmware tests are skipped. MSVC 2022 Build Tools are installed at `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools`, but `cl` is not on PATH. It needs `vcvars64.bat`. On 2026-10-12 the current sources built and passed with `cl /std:c++14 /W4 /WX`, both the test exe and the `/LD` DLL.

**Files:**
- Modify: `tests/test_firmware.py:1-38` (docstring, compiler discovery, `compile_cpp`)
- Modify: `README.md` (two lines about g++/clang++)

**Interfaces:**
- Produces: `COMPILER: tuple[str, str] | None`, which is `("gcc", path)` or `("msvc", path to vcvars64.bat)`, and `compile_cpp(sources, out: Path, shared=False)`, unchanged in signature. Later tasks keep calling `compile_cpp`.

- [ ] **Step 1: See the skip.**

Run: `.venv\Scripts\python -m pytest tests/test_firmware.py -v -rs`
Expected: 7 skipped, "no C++ compiler (g++ / clang++) on PATH".

- [ ] **Step 2: Replace the compiler discovery and `compile_cpp`.**

In `tests/test_firmware.py`, change the docstring's last line to `Needs a C++ compiler: g++ / clang++ on PATH, or Visual Studio C++ build tools (found with vswhere); skipped without one.`. Add `import os` to the imports. Replace the lines from `CXX = ...` through the end of `compile_cpp` with:

```python
def find_compiler():
    """("gcc", path) for g++ / clang++ on PATH, ("msvc", vcvars64.bat) for Visual Studio, or None."""
    for name in ("g++", "clang++", "c++"):
        if shutil.which(name):
            return "gcc", shutil.which(name)
    if sys.platform != "win32":
        return None
    vswhere = (Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
               / "Microsoft Visual Studio" / "Installer" / "vswhere.exe")
    if not vswhere.exists():
        return None
    found = subprocess.run([str(vswhere), "-products", "*", "-latest", "-requires",
                            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property",
                            "installationPath"], capture_output=True, text=True).stdout.strip()
    vcvars = Path(found) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat" if found else None
    return ("msvc", str(vcvars)) if vcvars and vcvars.exists() else None


COMPILER = find_compiler()

pytestmark = pytest.mark.skipif(
    COMPILER is None, reason="no C++ compiler: g++ / clang++ on PATH or Visual Studio C++ build tools")

# RejectController::State
IDLE, FAULTED = 1, 6


def compile_cpp(sources, out: Path, shared=False):
    kind, tool = COMPILER
    if kind == "gcc":
        cmd = [tool, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-o", str(out), *map(str, sources)]
        if shared:
            cmd[1:1] = ["-shared"] + ([] if sys.platform == "win32" else ["-fPIC"])
        result = subprocess.run(cmd, capture_output=True, text=True)
    else:
        # vcvars64.bat puts cl and its headers/libs on PATH for this one command; objects land in cwd
        cl = ["cl", "/nologo", "/std:c++14", "/W4", "/WX", "/EHsc", *(["/LD"] if shared else []),
              f"/Fe{out}", *map(str, sources)]
        result = subprocess.run(f'call "{tool}" >nul 2>nul && {subprocess.list2cmdline(cl)}',
                                shell=True, cwd=out.parent, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr  # cl reports errors on stdout
```

(The `# RejectController::State` / `IDLE, FAULTED` lines already exist. Keep them once, right after `pytestmark`.)

- [ ] **Step 3: Run the firmware tests.**

Run: `.venv\Scripts\python -m pytest tests/test_firmware.py -v`
Expected: 7 passed. If the ctypes load fails with "module not found", the DLL is missing the MSVC runtime. Check that `vcruntime140.dll` is in `C:\Windows\System32` before changing any code.

- [ ] **Step 4: Run the whole suite.**

Run: `.venv\Scripts\python -m pytest`
Expected: 68 passed, 0 skipped.

- [ ] **Step 5: Update the README (Bulgarian).**

In `README.md`, in the `tests/` table row, replace `slave логиката (C++, компилира се с g++/clang++)` with `slave логиката (C++, компилира се с g++/clang++ или MSVC)`.

Replace the line `Тестове (без камера и Arduino; C++ частта се пропуска, ако няма g++/clang++):` with `Тестове (без камера и Arduino; C++ частта се компилира с g++/clang++ или с MSVC от Visual Studio Build Tools, който се намира автоматично; пропуска се само ако няма компилатор):`.

- [ ] **Step 6: Commit.**

```bash
git add tests/test_firmware.py README.md
git commit -m "Firmware tests: build with MSVC (via vswhere/vcvars64) when g++ is missing"
```

---

### Task 2: A hand in the plate box holds the strike

**Why (PR #2 review, Important; PR #1 review, Minor 3):**
- `find_part` returns `found` when a complete brick lies next to an oversized blob such as a hand. With auto-reject (on by default when the arm is connected), an NG brick is then struck while a hand is in the box. Spec §5 rules this out.
- During VERIFY, a hand (`too_large`) counts as "the part moved", so a reject is wrongly reported as verified.
- A single stray `empty` frame re-arms the trigger, which allows a second strike on the same part.

The fix does **not** change `find_part`'s status. The brick is still inspected, so training data and edge-clutter handling stay the same. `Search` gains a `crowded` flag, and the trigger refuses to strike while it is set.

**Files:**
- Modify: `cell/geometry.py` (`Search`, `find_part`)
- Modify: `cell/reject.py` (`RejectTrigger.update`, re-arm rule)
- Modify: `tools/inspect_test.py` (`inspect()`, trigger call)
- Test: `tests/test_geometry.py`, `tests/test_reject_trigger.py`

**Interfaces:**
- Produces: `Search(status: str, part: Part | None = None, crowded: bool = False)`. `RejectTrigger.update(status, verdict, part, actuator, crowded: bool = False)`. Module constant `cell.reject.EMPTY_FRAMES = 3`. The `inspect()` result dict gains the key `"crowded": bool`.

- [ ] **Step 1: Write the failing geometry tests.** Append to `tests/test_geometry.py`:

```python
def test_hand_next_to_a_complete_brick_marks_the_search_crowded():
    brick, hand = ((480, 360), (170, 85), 0), ((740, 360), (260, 240), 0)
    assert hand[1][0] * hand[1][1] > MAX_AREA
    search = find_part(image(brick, hand), PLATE)
    assert search.status == "found"   # still inspected: training data and the HMI are unchanged
    assert search.crowded             # but the trigger must not strike
    assert search.part.center == pytest.approx((480, 360), abs=2)


def test_single_brick_is_not_crowded():
    assert not find_part(image(((640, 360), (170, 85), 30)), PLATE).crowded
```

- [ ] **Step 2: Write the failing trigger tests.** In `tests/test_reject_trigger.py`, change the import to `from cell.reject import EMPTY_FRAMES, SETTLE_FRAMES, VERIFY_FRAMES, RejectTrigger`. Append:

```python
def test_hand_next_to_an_ng_part_holds_the_strike():
    t, arm = RejectTrigger(), FakeActuator()
    for _ in range(3 * SETTLE_FRAMES):
        t.update("found", "NG", Part((300, 200)), arm, crowded=True)
    assert not arm.jobs
    assert "hand" in t.message
    frames(t, arm, SETTLE_FRAMES, part=Part((300, 200)))  # hand gone: settles again, then strikes
    assert len(arm.jobs) == 1


def test_hand_during_verify_is_not_a_verified_reject():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, 2 * VERIFY_FRAMES, status="too_large", verdict=None)
    assert t.state == "VERIFY"
    frames(t, arm, VERIFY_FRAMES, part=Part((300, 200)))  # hand gone, the part never moved
    assert t.state == "FAILED"


def test_one_empty_frame_does_not_rearm():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, VERIFY_FRAMES, part=Part((310, 205)))  # REJECT_FAILED
    frames(t, arm, 1, status="empty", verdict=None)       # one bad frame from find_part
    frames(t, arm, 30, part=Part((310, 205)))
    assert t.state == "FAILED"
    assert len(arm.jobs) == 1
```

The three tests that re-arm with a single empty frame need `EMPTY_FRAMES` frames now:
- `test_part_knocked_away_is_verified_and_struck_once`: change `frames(t, arm, 1, status="empty", verdict=None)` to `frames(t, arm, EMPTY_FRAMES, status="empty", verdict=None)`.
- `test_part_pushed_but_still_in_box_waits_for_removal`: the same change.
- `test_unmoved_part_is_reject_failed`: the same change.

- [ ] **Step 3: Run the tests and watch them fail.**

Run: `.venv\Scripts\python -m pytest tests/test_geometry.py tests/test_reject_trigger.py -v`
Expected: an ImportError on `EMPTY_FRAMES` (collection error). After you temporarily define it: the `crowded` tests fail with `AttributeError`/`TypeError`, and `test_one_empty_frame_does_not_rearm` fails because 2 jobs were created.

- [ ] **Step 4: Implement `crowded` in `cell/geometry.py`.**

```python
@dataclass(frozen=True)
class Search:
    status: str                  # "found", "empty", "outside" (touches the box edge), "multiple",
                                 # "too_large" (bigger than any part: a hand or the arm)
    part: Part | None = None
    crowded: bool = False        # a too-large blob (a hand?) is in the box besides the found part
```

In `find_part`, change the two returns after the `if not inside:` block:

```python
    if len(inside) > 1:
        return Search("multiple", crowded=too_large)
    (cx, cy), (rw, rh), angle = cv2.minAreaRect(inside[0])
    if rw < rh:  # make the first side the long one
        rw, rh, angle = rh, rw, angle + 90
    return Search("found", Part((cx + x0, cy + y0), rw, rh, angle), crowded=too_large)
```

- [ ] **Step 5: Implement the trigger changes in `cell/reject.py`.**

Add this line to the docstring's state list:

```
any state: a hand (crowded / too_large) in the box holds the strike and pauses VERIFY
CLEARING / FAILED  --plate box empty for EMPTY_FRAMES frames-->  ARMED
```

The second line replaces the existing `CLEARING / FAILED  --plate box empty-->  ARMED` line.

Constants: add `EMPTY_FRAMES = 3  # consecutive empty frames before re-arming: one bad frame must not allow a second strike`.

In `__init__`, add `self._empty_frames = 0`. Replace `update` with:

```python
    def update(self, status: str, verdict: str | None, part, actuator, crowded: bool = False) -> None:
        """One camera frame. status: find_part() status; verdict: "OK"/"NG" when inspected;
        part: the located Part or None; actuator: anything with request_reject() -> job | None;
        crowded: a hand-sized blob is in the box besides the part (Search.crowded)."""
        if self.state == "ARMED":
            if crowded:
                self._history = []
                self.message = "hand in the plate box: strike held"
            elif status == "found" and verdict == "NG":
                self._history = (self._history + [part.center])[-SETTLE_FRAMES:]
                if len(self._history) == SETTLE_FRAMES and self._settled():
                    job = actuator.request_reject()
                    if job is None:
                        self.message = "NG part, but the arm is not ready"
                    else:
                        self._job, self._struck = job, part
                        self._go("STRIKING", "striking")
            else:
                self._history = []
                self.message = ""
        elif self.state == "STRIKING":
            if self._job.state == "done":
                self._verify_frames = 0
                self._go("VERIFY", "verifying the reject")
            elif self._job.state == "failed":
                self._go("FAILED", f"REJECT NOT DONE: {self._job.error}")
        elif self.state == "VERIFY":
            if crowded or status == "too_large":
                self._verify_frames = 0  # a hand hides or moves the part: judge once it has gone
                self.message = "hand in the plate box: waiting to verify"
                return
            self._verify_frames += 1
            if self._verify_frames >= VERIFY_FRAMES:  # let the part come to rest first
                if self._moved(status, part):
                    self._go("CLEARING", "reject verified, remove the part")
                else:
                    self._go("FAILED", "REJECT_FAILED: the part did not move")
        else:  # CLEARING or FAILED
            self._empty_frames = self._empty_frames + 1 if status == "empty" else 0
            if self._empty_frames >= EMPTY_FRAMES:
                self._go("ARMED", "")
```

In `_go`, also reset `self._empty_frames = 0`.

- [ ] **Step 6: Wire it into `tools/inspect_test.py`.**

In `inspect()`, add `"crowded": search.crowded` to both returned dicts:
- the NO INSPECTION dict: `{"verdict": "NO INSPECTION", ..., "status": search.status, "crowded": search.crowded, ...}`;
- the inspected dict: `"status": search.status, "crowded": search.crowded,`.

In `main()`, change the trigger call to:

```python
                    trigger.update(result["status"], result["verdict"], result.get("part"), actuator,
                                   crowded=result["crowded"])
```

- [ ] **Step 7: Run the tests.**

Run: `.venv\Scripts\python -m pytest tests/test_geometry.py tests/test_reject_trigger.py -v`, then `.venv\Scripts\python -m pytest`
Expected: all pass. The full suite gives 73 passed (68 + 2 geometry + 3 trigger).

- [ ] **Step 8: Commit.**

```bash
git add cell/geometry.py cell/reject.py tools/inspect_test.py tests/test_geometry.py tests/test_reject_trigger.py
git commit -m "Hold the strike while a hand is in the plate box; re-arm only after 3 empty frames"
```

---

### Task 3: Check the strike settings, and derive the timeouts from them

**Why (PR #1 review, Important 1 and Minor 1):**
- `MOVE_TIMEOUT_MS = 2500` (slave) and `DONE_TIMEOUT = 3.0` (master) are fixed, but `--speed` / `--accel` / `--strike-steps` are free CLI values.
- With AccelStepper's ramps plus the Modbus pauses, these slow settings fault although the arm is working:
  - 600 steps/s hits the master's DONE timeout (3.29 s for the full cycle);
  - 400 steps/s reaches the slave's MOVE_TIMEOUT on the outward stroke, leaving the arm over the plate.
- `--strike-steps 40000` wraps to −25536 in the 16-bit register and turns the arm the wrong way.

**Formula (the same on both sides):** for one stroke of |s| steps at speed v and acceleration a:
- trapezoid, when |s| ≥ v²/a: t = |s|/v + v/a;
- triangle, otherwise: t = 2·√(|s|/a).

The slave's stroke timeout is `1.5·t + 0.5 s`, where 1.5 covers the ~25 % of loop time the Modbus requests take, plus margin. The master's DONE timeout is `2 × stroke timeout + 0.5 s`, so the slave always reports its specific MOVE_TIMEOUT first. At the defaults (800, 2400, 12000): t = 0.533 s, stroke timeout = 1.3 s, DONE timeout = 3.1 s.

**Files:**
- Modify: `firmware/reject_cell/RejectController.h`, `firmware/reject_cell/RejectController.cpp`
- Modify: `tests/firmware/slave_sim.cpp`, `tests/firmware/reject_controller_test.cpp`
- Modify: `cell/actuator.py`, `tools/inspect_test.py`
- Modify: `docs/superpowers/specs/2026-10-11-duplo-inspection-design.md` (§3 layout, §7 timeouts)
- Test: `tests/test_actuator.py`, `tests/test_firmware.py`

**Interfaces:**
- Produces, C++: `static uint32_t RejectController::strokeTimeoutMs(long steps, float speed, float accel)`, `STROKE_SLACK = 1.5f`, `STROKE_MARGIN_MS = 500`. The sim exports `uint32_t sim_stroke_timeout_ms(int steps, int speed, int accel)`.
- Produces, Python (`cell/actuator.py`): `stroke_time(steps, speed, accel) -> float`, `move_timeout(steps, speed, accel) -> float`, `check_strike(strike_steps, speed, accel) -> None` (raises `ValueError`), `ActuatorMaster.done_timeout: float`. The constant `DONE_TIMEOUT` is **removed**.

- [ ] **Step 1: Write the failing C++ tests.**

In `tests/firmware/reject_controller_test.cpp`, add these before `main`:

```cpp
static void test_stroke_timeout_follows_the_settings() {
  uint32_t fast = RejectController::strokeTimeoutMs(800, 2400, 12000);
  CHECK(fast >= 1290 && fast <= 1310);  // 1.5 x 0.533 s + 0.5 s
  CHECK(RejectController::strokeTimeoutMs(-800, 2400, 12000) == fast);
  CHECK(RejectController::strokeTimeoutMs(800, 300, 12000) > 4000);
  CHECK(RejectController::strokeTimeoutMs(800, 0, 0) > 0);  // bad registers: no division by zero
}

static void test_slow_strike_is_not_a_move_timeout() {
  Rig rig;
  rig.regs.holding[reg::MAX_SPEED] = 300;  // 2.7 s per stroke: over the old fixed 2.5 s
  rig.run(100);
  rig.setRequest(true);
  runUntilDone(rig, 8000);
  CHECK(rig.done());
  CHECK(!rig.faulted());
  CHECK(rig.cycles() == 1);
}
```

Register both in `main`'s table:

```cpp
      {"stroke_timeout_follows_the_settings", test_stroke_timeout_follows_the_settings},
      {"slow_strike_is_not_a_move_timeout", test_slow_strike_is_not_a_move_timeout},
```

`test_stalled_motor_times_out` assumed 2.5 s. At the defaults the timeout is now ~1.3 s, so change its body to:

```cpp
  Rig rig;
  rig.run(100);
  rig.arm.stalled = true;
  rig.setRequest(true);
  rig.run(1250);
  CHECK(rig.busy());
  rig.run(100);
  CHECK(rig.faulted());
  CHECK(rig.faultCode() == FAULT_MOVE_TIMEOUT);
  CHECK(rig.arm.halts == 1);
```

Change the comment above `slowStrike` to: `// a strike at full speed (~0.67 s on FakeArm, which has no ramps) is shorter than the watchdog, so slow it down to catch it mid-move`.

- [ ] **Step 2: Write the failing Python tests.**

In `tests/test_actuator.py`:
- In the import, replace `DONE_TIMEOUT` with `move_timeout, stroke_time`.
- In `test_no_done_times_out`, change the last line to `assert clock() - 100.3 == pytest.approx(master.done_timeout + 0.05, abs=0.06)`.
- Append:

```python
def test_stroke_time_trapezoid_and_triangle():
    assert stroke_time(800, 2400, 12000) == pytest.approx(800 / 2400 + 2400 / 12000)
    assert stroke_time(100, 2400, 12000) == pytest.approx(2 * (100 / 12000) ** 0.5)  # never at full speed
    assert stroke_time(-800, 2400, 12000) == stroke_time(800, 2400, 12000)


def test_done_timeout_follows_the_strike_settings(clock, slave):
    fast = ActuatorMaster(slave, clock=clock)
    slow = ActuatorMaster(slave, speed=600, clock=clock)
    assert fast.done_timeout == pytest.approx(2 * move_timeout(800, 2400, 12000) + 0.5)
    assert slow.done_timeout > fast.done_timeout


def test_slow_strike_is_not_a_timeout(clock, slave):
    slave.strike_time = 3.3  # 600 steps/s on the real stepper, ramps and Modbus pauses included
    m = ActuatorMaster(slave, speed=600, clock=clock)
    run(m, clock, 0.3)
    assert reject(m, clock, seconds=8.0).state == "done", m.fault


@pytest.mark.parametrize("kwargs", [dict(strike_steps=40000), dict(strike_steps=0), dict(speed=0),
                                    dict(speed=5000), dict(accel=0), dict(accel=70000)])
def test_rejects_out_of_range_strike_settings(clock, slave, kwargs):
    with pytest.raises(ValueError):
        ActuatorMaster(slave, clock=clock, **kwargs)
```

In `tests/test_firmware.py`:
- Add `move_timeout` to the `cell.actuator` import.
- In the `lib` fixture, after the existing loops, add:

```python
    lib.sim_stroke_timeout_ms.argtypes = [ctypes.c_int] * 3
    lib.sim_stroke_timeout_ms.restype = ctypes.c_uint32
```

- Append:

```python
@pytest.mark.parametrize("steps, speed, accel", [(800, 2400, 12000), (-600, 1200, 12000),
                                                 (800, 300, 12000), (100, 4000, 2000)])
def test_master_and_slave_agree_on_the_stroke_timeout(lib, steps, speed, accel):
    assert lib.sim_stroke_timeout_ms(steps, speed, accel) == pytest.approx(
        move_timeout(steps, speed, accel) * 1000, abs=2)
```

- [ ] **Step 3: Run the tests and watch them fail.**

Run: `.venv\Scripts\python -m pytest tests/test_actuator.py tests/test_firmware.py -v`
Expected: test_actuator fails at collection (`move_timeout` does not exist), and the firmware build fails (`strokeTimeoutMs` is not a member).

- [ ] **Step 4: Implement it in the firmware.**

In `RejectController.h`, replace the `MOVE_TIMEOUT_MS` line with:

```cpp
  // stroke timeout = ideal stroke time (trapezoidal speed profile) x STROKE_SLACK + STROKE_MARGIN_MS;
  // the slack covers the ~25 % of loop() the Modbus requests take. cell/actuator.py mirrors this.
  static constexpr float STROKE_SLACK = 1.5f;
  static const uint32_t STROKE_MARGIN_MS = 500;
  static uint32_t strokeTimeoutMs(long steps, float speed, float accel);
```

and add the private member `uint32_t moveTimeoutMs_ = 0;` after `strokeStart_`.

In `RejectController.cpp`, add `#include <math.h>` under the existing include, and add:

```cpp
uint32_t RejectController::strokeTimeoutMs(long steps, float speed, float accel) {
  const float s = (float)(steps < 0 ? -steps : steps);
  if (speed < 1) speed = 1;  // a zero register must not divide by zero
  if (accel < 1) accel = 1;
  const float ramp = speed * speed / accel;  // steps spent speeding up and slowing down from full speed
  const float t = s >= ramp ? s / speed + speed / accel : 2 * sqrtf(s / accel);
  return (uint32_t)(STROKE_SLACK * t * 1000.0f) + STROKE_MARGIN_MS;
}
```

In `update()`, change `} else if (now - strokeStart_ > MOVE_TIMEOUT_MS) {` to `} else if (now - strokeStart_ > moveTimeoutMs_) {`.

Replace `startStroke` with:

```cpp
void RejectController::startStroke(long target, State next, uint32_t now) {
  moveTimeoutMs_ = strokeTimeoutMs(target - arm_.position(), regs_.holding[reg::MAX_SPEED],
                                   regs_.holding[reg::ACCELERATION]);
  arm_.moveTo(target);
  strokeStart_ = now;
  state_ = next;
}
```

In `tests/firmware/slave_sim.cpp`, append:

```cpp
API uint32_t sim_stroke_timeout_ms(int steps, int speed, int accel) {
  return RejectController::strokeTimeoutMs(steps, (float)speed, (float)accel);
}
```

- [ ] **Step 5: Implement it on the master.**

In `cell/actuator.py`:
- Add `import math`.
- Delete the `DONE_TIMEOUT = 3.0` line.
- Below `RELEASE_TIMEOUT`, add:

```python
STROKE_SLACK = 1.5        # must match RejectController::STROKE_SLACK
STROKE_MARGIN = 0.5       # s, must match RejectController::STROKE_MARGIN_MS
DONE_MARGIN = 0.5         # s the master waits beyond the slave's own two stroke timeouts
MAX_STEP_RATE = 4000      # steps/s: about what AccelStepper can step on a 16 MHz Mega


def stroke_time(steps: int, speed: float, accel: float) -> float:
    """Seconds for one stroke of |steps| at `speed` steps/s, ramping at `accel` steps/s^2."""
    steps = abs(steps)
    ramp = speed * speed / accel  # steps spent speeding up and slowing down from full speed
    if steps >= ramp:
        return steps / speed + speed / accel
    return 2 * math.sqrt(steps / accel)  # never reaches full speed


def move_timeout(steps: int, speed: float, accel: float) -> float:
    """The slave's timeout for one stroke (RejectController::strokeTimeoutMs), in seconds."""
    return STROKE_SLACK * stroke_time(steps, speed, accel) + STROKE_MARGIN


def check_strike(strike_steps: int, speed: int, accel: int):
    """Raises ValueError for settings the 16-bit registers or the Mega cannot carry."""
    if not 1 <= abs(strike_steps) <= 0x7FFF:
        raise ValueError(f"strike steps {strike_steps}: must be 1..32767 either way")
    if not 1 <= speed <= MAX_STEP_RATE:
        raise ValueError(f"speed {speed}: must be 1..{MAX_STEP_RATE} steps/s")
    if not 1 <= accel <= 0xFFFF:
        raise ValueError(f"acceleration {accel}: must be 1..65535 steps/s^2")
```

In `ActuatorMaster.__init__`, add these as the first two lines of the body:

```python
        check_strike(strike_steps, speed, accel)
        self.done_timeout = 2 * move_timeout(strike_steps, speed, accel) + DONE_MARGIN
```

In `_advance`, replace both uses of `DONE_TIMEOUT`:

```python
            elif now - self._request_at > self.done_timeout:
                self._latch(f"ACTUATOR_TIMEOUT: no DONE within {self.done_timeout:.1f} s", now)
```

In the module docstring, change the timeout sentence so that it ends: `...the master faults if the slave's heartbeat stops changing for 500 ms. Stroke and DONE timeouts follow the strike settings (move_timeout()).`

- [ ] **Step 6: Check the CLI values before opening the port.**

In `tools/inspect_test.py`, add `check_strike` to the `cell.actuator` import. In `main()`, replace the `if not args.no_motor:` block's first two lines with:

```python
    if not args.no_motor:
        steps = -args.strike_steps if args.reverse else args.strike_steps
        try:
            check_strike(steps, args.speed, args.accel)
        except ValueError as exc:
            parser.error(str(exc))
        master = ActuatorMaster(ModbusBus.serial(args.port), strike_steps=steps, speed=args.speed, accel=args.accel)
```

(Task 6 moves the `ActuatorThread(...).start()` line. Leave it in place for now.)

- [ ] **Step 7: Run the tests.**

Run: `.venv\Scripts\python -m pytest`
Expected: 0 failed, 0 skipped. That is 73 from before, plus 3 + 6 new cases in `test_actuator.py` and 4 in `test_firmware.py`, so 86 passed. The C++ checks run inside the one existing `test_controller_unit_tests`.

- [ ] **Step 8: Update the spec.**

In `docs/superpowers/specs/2026-10-11-duplo-inspection-design.md`:
- §3 code layout: replace `firmware/reject_arm/reject_arm.ino` with `firmware/reject_cell/   reject_cell.ino + RejectController (Arduino-free, unit-tested on the PC)`.
- §3: replace `` `request_reject()` / `poll()` `` with `` `request_reject()` / `step()` (run by `ActuatorThread`) ``.
- §7: replace the `**Timeouts (master):** ...` paragraph with:

```
**Timeouts (master):** BUSY within 300 ms of REQ; DONE within 2 × the slave's stroke timeout + 0.5 s
(3.1 s at the defaults); slave heartbeat change within 500 ms. **Slave:** each stroke must finish within
1.5 × its ideal time for the set steps, speed and acceleration + 0.5 s (1.3 s at the defaults) →
stop motor, FAULT_CODE=2; master heartbeat unchanged for 1 s → stop motor, FAULT_CODE=1. The master
refuses strike settings outside |steps| 1..32767, speed 1..4000 steps/s, acceleration 1..65535.
```

- §7: replace the bullet that starts `- A full-speed strike (~0.7 s)` with:

```
- A full strike at the defaults takes ~1.1-1.3 s on the real stepper (ramps plus the Modbus pauses),
  longer than the 1 s watchdog: a master that dies mid-strike leaves the arm halted part-way.
```

- [ ] **Step 9: Compile the sketch for the Mega**, if `arduino-cli` is installed. If it is not, write that in the task report; do not skip silently.

Run: `arduino-cli compile --fqbn arduino:avr:mega --warnings all firmware/reject_cell`
Expected: success, with no warnings from the sketch's own files.

- [ ] **Step 10: Commit.**

```bash
git add firmware/reject_cell tests/firmware cell/actuator.py tools/inspect_test.py tests/test_actuator.py tests/test_firmware.py docs/superpowers/specs/2026-10-11-duplo-inspection-design.md
git commit -m "Derive stroke and DONE timeouts from the strike settings; range-check them"
```

---

### Task 4: After a restart away from home, report NOT_HOME instead of silently re-homing

**Why (PR #1 review, Important 2):**
- Home is wherever the stick is at power-on.
- Opening the serial port resets the Mega, and so does pymodbus reopening it after an `OSError`.
- A full cycle (~1.2 s) is longer than the 1 s watchdog, so an app that exits mid-strike usually leaves the stick halted off home.
- The next launch then silently takes that position as home: every strike after that is offset, and the resting stick may sit in the camera's plate box.

The fix (agreed with the owner on 2026-10-12: a quick demo, so no limit switch):
- The firmware keeps a one-byte EEPROM flag, "away from home". It is set when a stroke leaves home and cleared when the stick arrives home again.
- At boot, a set flag puts the slave in FAULTED with the new code `FAULT_NOT_HOME = 3`.
- The master latches that fault. The operator puts the stick home by hand and presses **R**, which means "the stick is home". X does not clear NOT_HOME, so a plain fault reset cannot skip that check.
- **Only a restart while the stick was away asks for R.** Normal start-ups, including the reset that opening the port causes, and normal strikes need no key.
- In `tools/inspect_test.py`, R was the real-time toggle. That moves to **T**.

Wear: 2 EEPROM writes per strike, with 100k cycles per cell, gives ~50k strikes. That is fine for this cell, and the code comment says so.

**Files:**
- Modify: `firmware/reject_cell/RejectController.h`, `RejectController.cpp`, `reject_cell.ino`
- Modify: `tests/firmware/fake_arm.h`, `tests/firmware/slave_sim.cpp`, `tests/firmware/reject_controller_test.cpp`
- Modify: `cell/actuator.py` (`SLAVE_FAULTS`, `reset`, new `confirm_home`, `_latch`)
- Modify: `tools/inspect_test.py` (keys: T real-time, R stick home)
- Modify: spec §5, §7 and §8, and `README.md`
- Test: `tests/test_firmware.py`, `tests/test_actuator.py`

**Interfaces:**
- Consumes: `startStroke` from Task 3.
- Produces, C++: `class Persist { virtual bool awayFromHome() = 0; virtual void setAwayFromHome(bool) = 0; }`, `RejectController(Registers&, Actuator&, Persist&)`, `FAULT_NOT_HOME = 3`. `begin()` now resets **all** controller state, so calling it again acts as a power cycle. Test side: `FakePersist {bool away; int writes;}` and `FakeArm::reboot()`. The sim exports `sim_reboot(Sim*)` and `sim_away(Sim*)`.
- Produces, Python:
  - `SLAVE_FAULT_NOT_HOME = 3` and `SLAVE_FAULTS[3]`, which starts with `"NOT_HOME"`;
  - `ActuatorMaster.confirm_home()`, which clears only a latched NOT_HOME, and `ActuatorMaster.reset()`, which now ignores NOT_HOME;
  - `ActuatorThread.confirm_home()`.

- [ ] **Step 1: Write the failing C++ tests.** In `tests/firmware/fake_arm.h`, add this before `struct Rig`:

```cpp
// The EEPROM flag, in memory; counts real writes (EEPROM.update skips unchanged values)
struct FakePersist : public Persist {
  bool away = false;
  int writes = 0;
  bool awayFromHome() override { return away; }
  void setAwayFromHome(bool v) override {
    if (v != away) writes++;
    away = v;
  }
};
```

Add `reboot()` to `FakeArm`'s public section:

```cpp
  // the Mega restarts: AccelStepper counts from 0 again, wherever the stick physically is
  void reboot() {
    pos_ = target_ = 0;
    credit_ = 0;
  }
```

In `Rig`, declare `FakePersist persist;` after `FakeArm arm;`, and change the controller to `RejectController ctrl{regs, arm, persist};`.

In `reject_controller_test.cpp`, add these tests and register them in `main`:

```cpp
static void test_strike_marks_the_arm_away_until_home() {
  Rig rig;
  rig.run(100);
  rig.setRequest(true);
  rig.run(50);
  CHECK(rig.persist.away);           // out on a stroke: a reboot now would lose home
  runUntilDone(rig);
  CHECK(!rig.persist.away);          // back home
  CHECK(rig.persist.writes == 2);    // one EEPROM write each way, not one per loop
}

static void test_fault_mid_strike_keeps_the_arm_marked_away() {
  Rig rig;
  slowStrike(rig);
  rig.run(100);
  rig.setRequest(true);
  rig.run(150);
  rig.masterBeating = false;
  rig.run(1100);
  CHECK(rig.faulted());
  CHECK(rig.persist.away);
}

static void test_boot_away_from_home_faults_until_reset() {
  Rig rig;
  rig.persist.away = true;           // the Mega restarted while the stick was out
  rig.arm.reboot();
  rig.ctrl.begin(rig.now);
  rig.run(200);
  CHECK(rig.faulted());
  CHECK(rig.faultCode() == FAULT_NOT_HOME);
  rig.setRequest(true);
  rig.run(100);
  CHECK(rig.arm.maxPos == 0);        // no strike from an unknown position
  rig.setRequest(false);
  rig.setReset(true);                // the operator has put the stick home by hand
  rig.run(1);
  CHECK(rig.ready());
  CHECK(!rig.persist.away);
}
```

```cpp
      {"strike_marks_the_arm_away_until_home", test_strike_marks_the_arm_away_until_home},
      {"fault_mid_strike_keeps_the_arm_marked_away", test_fault_mid_strike_keeps_the_arm_marked_away},
      {"boot_away_from_home_faults_until_reset", test_boot_away_from_home_faults_until_reset},
```

- [ ] **Step 2: Write the failing Python test against the real slave logic.**

In `tests/firmware/slave_sim.cpp`, add `FakePersist persist;` to `Sim` after `FakeArm arm;`, and change the controller to `RejectController ctrl{regs, arm, persist};`. Append:

```cpp
// power-cycle the Mega: registers and the step counter start from zero, the EEPROM flag survives
API void sim_reboot(Sim* s) {
  s->regs = Registers{};
  s->arm.reboot();
  s->ctrl.begin(s->now);
}
API int sim_away(Sim* s) { return s->persist.away; }
```

In `tests/test_firmware.py`, add `"sim_reboot", "sim_away"` to the name tuple in the `lib` fixture (neither takes extra args), and append:

```python
def test_reboot_away_from_home_is_latched_for_the_operator(slave, clock):
    m = connect(slave, clock, speed=400)  # 800 steps out take 2 s
    job = m.request_reject()
    run(m, clock, 0.5, slave_advance=slave.advance)
    assert slave.position() > 0 and slave.away()
    slave.reboot()  # USB brown-out, or pymodbus reopening the port
    run(m, clock, 0.3, slave_advance=slave.advance)
    assert job.state == "failed"
    assert m.phase is Phase.FAULT and "NOT_HOME" in m.fault
    assert m.request_reject() is None
    m.reset()  # X is not enough: someone must say the stick is home
    run(m, clock, 0.3, slave_advance=slave.advance)
    assert m.phase is Phase.FAULT
    # the operator puts the stick home by hand, then presses R
    m.confirm_home()
    assert run(m, clock, 1.0, slave_advance=slave.advance, until=lambda: m.ready)
    assert not slave.away()
    assert reject(m, slave, clock, seconds=8.0).state == "done", m.fault


def test_reboot_at_home_is_harmless(slave, clock):
    m = connect(slave, clock)
    assert reject(m, slave, clock).state == "done"
    run(m, clock, 0.1, slave_advance=slave.advance)
    slave.reboot()
    assert run(m, clock, 1.0, slave_advance=slave.advance, until=lambda: m.ready)
    assert m.fault == ""  # no R needed: the stick was home
```

Append to `tests/test_actuator.py`:

```python
def test_not_home_needs_confirm_home_not_reset(master, slave, clock):
    slave.trip(3)  # the arm restarted while the stick was out
    run(master, clock, 0.1)
    assert "NOT_HOME" in master.fault and "R" in master.fault
    master.reset()
    run(master, clock, 0.2)
    assert master.phase is Phase.FAULT and slave.resets == 0
    master.confirm_home()
    run(master, clock, 0.1)
    assert slave.resets == 1
    assert master.phase is Phase.IDLE and master.fault == ""


def test_confirm_home_does_not_clear_other_faults(master, slave, clock):
    slave.trip(2)
    run(master, clock, 0.1)
    master.confirm_home()
    run(master, clock, 0.2)
    assert master.phase is Phase.FAULT and slave.resets == 0
```

- [ ] **Step 3: Run the tests and watch them fail.**

Run: `.venv\Scripts\python -m pytest tests/test_firmware.py -v`
Expected: a compile error, because `Persist` is not declared.

- [ ] **Step 4: Implement it in the controller.**

In `RejectController.h`:
- Change the fault enum to `enum FaultCode : uint16_t { FAULT_NONE = 0, FAULT_COMM_LOST = 1, FAULT_MOVE_TIMEOUT = 2, FAULT_NOT_HOME = 3 };`.
- After the `Actuator` class, add:

```cpp
// One flag that survives a reset (EEPROM on the Mega): the stick is away from home. Home is the
// power-on position, so a restart while away would silently move home; the flag catches that.
class Persist {
 public:
  virtual bool awayFromHome() = 0;
  virtual void setAwayFromHome(bool away) = 0;
};
```

- Change the constructor to `RejectController(Registers& regs, Actuator& arm, Persist& persist) : regs_(regs), arm_(arm), persist_(persist) {}`, add the member `Persist& persist_;` after `Actuator& arm_;`, and change the `begin` comment to `// Power-on: resets all state. The power-on position is home (no limit switch), unless the flag says the stick was away.`.

In `RejectController.cpp`, replace `begin` with:

```cpp
void RejectController::begin(uint32_t now) {
  for (bool& c : regs_.coils) c = false;
  regs_.holding[reg::MASTER_HEARTBEAT] = 0;
  regs_.holding[reg::STRIKE_STEPS] = (uint16_t)DEFAULT_STRIKE_STEPS;
  regs_.holding[reg::MAX_SPEED] = DEFAULT_MAX_SPEED;
  regs_.holding[reg::ACCELERATION] = DEFAULT_ACCELERATION;
  regs_.input[reg::SLAVE_HEARTBEAT] = 0;
  home_ = arm_.position();
  strikeTarget_ = home_;
  strokeStart_ = now;
  moveTimeoutMs_ = 0;
  lastBeat_ = now;
  lastMasterBeat_ = 0;
  masterBeatAt_ = now;
  masterSeen_ = false;
  requestArmed_ = false;
  prevReset_ = false;
  cycleCount_ = 0;
  faultCode_ = FAULT_NONE;
  state_ = WAIT_MASTER;
  if (persist_.awayFromHome()) {  // restarted mid-stroke: this "home" is wherever the stick stopped
    faultCode_ = FAULT_NOT_HOME;
    state_ = FAULTED;
  }
  publish();
}
```

In `update()`, set the flag when leaving home and clear it on arrival:
- `IDLE` case: put `persist_.setAwayFromHome(true);` right before `startStroke(strikeTarget_, STRIKE_OUT, now);`.
- `STRIKE_BACK` arrival: change it to `} else if (state_ == STRIKE_BACK) { persist_.setAwayFromHome(false); cycleCount_++; state_ = DONE;`.
- `RETURNING` arrival: change the final `else` to `} else { persist_.setAwayFromHome(false); state_ = IDLE; }`.
- `FAULTED` reset with `arm_.position() == home_`: change it to `persist_.setAwayFromHome(false); state_ = IDLE;`. On a NOT_HOME reset, the operator vouches that the stick is home.

- [ ] **Step 5: Wire the EEPROM into the sketch.** In `reject_cell.ino`, add `#include <EEPROM.h>` with the other includes, and before `StepperArm arm;` add:

```cpp
// Byte 0: 1 while the stick is away from home. EEPROM.update writes only on change: 2 writes per
// strike, ~50k strikes for the cell's 100k-cycle endurance (fine for this cell).
class EepromPersist : public Persist {
 public:
  bool awayFromHome() override { return EEPROM.read(0) == 1; }  // a fresh EEPROM reads 0xFF
  void setAwayFromHome(bool away) override { EEPROM.update(0, away ? 1 : 0); }
};

EepromPersist persist;
```

Change the controller to `RejectController controller(regs, arm, persist);`.

- [ ] **Step 6: Implement it on the master.** In `cell/actuator.py`:

```python
SLAVE_FAULT_COMM_LOST, SLAVE_FAULT_NOT_HOME = 1, 3
SLAVE_FAULTS = {0: "NONE", 1: "COMM_LOST", 2: "MOVE_TIMEOUT",
                3: "NOT_HOME (restarted mid-strike): put the stick home by hand, press R"}
```

(This replaces the existing `SLAVE_FAULT_COMM_LOST = 1` and `SLAVE_FAULTS` lines.)

In `ActuatorMaster.__init__`, add `self._latched_code = None  # slave fault code behind a latched fault`.

Change `_latch` to take the slave's code, and pass it where a slave fault is latched:

```python
    def _latch(self, reason: str, now, slave_code=None):
        self.fault = reason
        self._latched_code = slave_code
        ...  # rest unchanged
```

In `_advance` and in `_connect`, both `self._latch(f"ACTUATOR_FAULT: arm reports ...", now)` calls become `self._latch(f"ACTUATOR_FAULT: arm reports {SLAVE_FAULTS.get(s.fault_code, s.fault_code)}", now, s.fault_code)`.

Replace `reset` and add `confirm_home`:

```python
    def reset(self):
        """Operator reset of a latched fault (X). NOT_HOME needs confirm_home() (R) instead."""
        if self.phase is Phase.FAULT and self._latched_code != SLAVE_FAULT_NOT_HOME:
            self._reset_requested = True

    def confirm_home(self):
        """Operator (R): the stick is back home by hand, after the arm restarted mid-strike."""
        if self.phase is Phase.FAULT and self._latched_code == SLAVE_FAULT_NOT_HOME:
            self._reset_requested = True
```

The existing FAULT branch in `_advance` already pulses RESET_FAULT when `_reset_requested` is set. The slave then clears NOT_HOME, takes the current position as home and clears the EEPROM flag (Step 4).

In `ActuatorThread`, add:

```python
    def confirm_home(self):
        with self._lock:
            self.master.confirm_home()
```

- [ ] **Step 6b: Keys in `tools/inspect_test.py`.**
- Docstring: change `R      real-time mode on/off: inspect every frame (starts on)` to `T      real-time mode on/off: inspect every frame (starts on)`, and add `R      the stick is back home: only asked for after the arm restarted mid-strike (NOT_HOME)`.
- In the key handling, change `if key == ord("r"):` to `if key == ord("t"):`, and add this branch next to the `x` branch:

```python
            elif key == ord("r") and actuator is not None:
                actuator.confirm_home()
                status = "stick home confirmed"
```

- Footer: `"T real-time | A auto | SPACE snapshot | K strike | R stick home | X reset | L re-lock | Q quit"`.

- [ ] **Step 7: Run the tests.**

Run: `.venv\Scripts\python -m pytest`
Expected: all pass, with 0 skipped.

- [ ] **Step 8: Update the docs.**

Spec §7:
- In the register table, change `FAULT_CODE` meaning to `0 none, 1 comm lost, 2 move timeout, 3 not home`.
- Replace the bullet `- On connect the master clears REQ_REJECT, ... latched for the operator.` with:

```
- On connect the master clears REQ_REJECT, writes the strike parameters and auto-resets a
  COMM_LOST fault, retrying until the slave has seen the new heartbeat. Any other slave fault is
  latched for the operator. (Opening the port resets the Mega, so a leftover COMM_LOST is rare.)
- Home is the power-on position, so the firmware keeps an EEPROM flag "away from home", set when a
  stroke leaves home and cleared on arrival. Booting with it set gives FAULT_CODE=3 NOT_HOME: no
  strike until the operator has put the stick home and pressed R (X does not clear it). Only a
  restart while away asks for R; normal start-ups and strikes do not.
```

Spec §5: change `──R key (reset) and condition cleared──►` to `──X key (reset; R = "stick is home" for NOT_HOME) and condition cleared──►`.

In the §8 fault table, add the row `| ACTUATOR NOT_HOME | arm restarted while away from home (EEPROM flag) | FAULT, operator puts the stick home by hand, presses R |`.

In `README.md`:
- In the `tools/inspect_test.py` row, change `` `R` реално време `` to `` `T` реално време ``, and add `` `R` пръчката е върната в начална позиция (само след рестарт по време на удар) `` before `` `X` нулиране на грешка ``.
- After the firmware/wiring paragraph, add:

```
Ако Arduino се рестартира, докато пръчката е навън (срив по време на удар, прекъснат USB),
рамото съобщава `NOT_HOME` и не приема удари: върнете пръчката в началната позиция на ръка
(ако драйверът я държи, изключете захранването му за момент) и натиснете `R`. При нормален
старт и нормални удари `R` не е нужно.
```

- [ ] **Step 9: Compile the sketch for the Mega**, as in Task 3 Step 9. Record the RAM and flash figures in the task report.

- [ ] **Step 10: Commit.**

```bash
git add firmware/reject_cell tests/firmware tests/test_firmware.py tests/test_actuator.py cell/actuator.py tools/inspect_test.py docs/superpowers/specs/2026-10-11-duplo-inspection-design.md README.md
git commit -m "NOT_HOME after a restart mid-strike, confirmed with R; real-time toggle moves to T"
```

---

### Task 5: A broken or stalled actuator thread can no longer hide

**Why (PR #1 review, Important 4 and Minors 2 and 4):**
- Any exception other than `BusError` in `step()` kills `ActuatorThread` silently. `ready` then stays True from the last status, jobs stay pending forever, `RejectTrigger` sits in STRIKING, and the HMI shows "arm: idle".
- If this process stalls for more than 1 s (for example while RealSense starts), the slave's COMM_LOST arrives while the master is IDLE. The master then latches it for a manual X, while the same fault found at connect is auto-reset.
- `RejectJob._finish` sets `state` before `error`, so the camera thread can show an empty reason.

**Files:**
- Modify: `cell/actuator.py` (`RejectJob._finish`, `ActuatorMaster.ready`, `step`, `_advance`, new `internal_error`, `ActuatorThread._run`)
- Test: `tests/test_actuator.py`

**Interfaces:**
- Produces: `ActuatorMaster.internal_error(reason: str) -> None`. `ActuatorMaster.ready` is now also False when no status has been read in the last `HEARTBEAT_TIMEOUT` s.

- [ ] **Step 1: Write the failing tests.** Append to `tests/test_actuator.py`:

```python
def wait_until(cond, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not cond() and time.monotonic() < deadline:
        time.sleep(0.01)
    return cond()


def test_not_ready_without_a_recent_status(master, clock):
    assert master.ready
    clock.advance(1.0)  # no step() ran: e.g. the actuator thread is stuck
    assert not master.ready
    assert master.request_reject() is None


def test_thread_survives_a_bug_in_step_and_reports_it():
    slave = FakeSlave(time.monotonic)
    slave.strike_time = 1.0
    actuator = ActuatorThread(ActuatorMaster(slave), period=0.01).start()
    try:
        assert wait_until(lambda: actuator.master.ready)
        job = actuator.request_reject()

        def broken():
            raise KeyError("unexpected response")

        slave.read_status = broken
        assert wait_until(lambda: job.state != "pending")
        assert job.state == "failed" and "ACTUATOR_INTERNAL" in job.error
        assert actuator.master.phase is Phase.FAULT
        assert "ACTUATOR_INTERNAL" in actuator.describe()
        assert actuator._thread.is_alive()
    finally:
        actuator.stop()


def test_slave_comm_lost_after_a_master_stall_recovers_by_itself(master, slave, clock):
    slave.trip(1)  # this process stalled > 1 s; the link itself is fine again
    run(master, clock, 0.2)
    assert slave.resets == 1
    assert master.phase is Phase.IDLE and master.fault == ""
    assert reject(master, clock).state == "done"


def test_slave_comm_lost_mid_cycle_fails_the_job_then_recovers(master, slave, clock):
    slave.strike_time = 2.0
    job = master.request_reject()
    run(master, clock, 0.1)
    slave.trip(1)
    run(master, clock, 0.2)
    assert job.state == "failed" and "COMM_LOST" in job.error
    assert master.phase is Phase.IDLE
```

- [ ] **Step 2: Run them and watch them fail.**

Run: `.venv\Scripts\python -m pytest tests/test_actuator.py -v`
Expected:
- `test_not_ready_without_a_recent_status` fails, because it is still ready.
- The thread test fails: the job stays pending and `wait_until` returns False.
- The two COMM_LOST tests fail, because the phase is FAULT.

- [ ] **Step 3: Implement it.** In `cell/actuator.py`:

```python
    def _finish(self, error: str = ""):
        self.error = error  # before state: another thread may read error as soon as state changes
        self.state = "failed" if error else "done"
```

In `ActuatorMaster.__init__`, add `self._status_at = float("-inf")`.

```python
    @property
    def ready(self) -> bool:
        fresh = self.clock() - self._status_at <= HEARTBEAT_TIMEOUT
        return (self.phase is Phase.IDLE and self.job is None and fresh
                and bool(self.status and self.status.ready))
```

In `step()`, record the read time right after the successful read:

```python
            status = self.status = self.bus.read_status()
            self._status_at = now
```

In `_advance`, insert this branch between the `elif self.phase is Phase.FAULT:` block and `elif s.fault:`:

```python
        elif s.fault and s.fault_code == SLAVE_FAULT_COMM_LOST:
            # the slave lost our heartbeat (this process stalled > 1 s) but the link is fine again:
            # recover exactly as a fresh connect does
            self._fail_job("ACTUATOR_COMM_LOST: the arm lost the master heartbeat")
            self.fault = "ACTUATOR_COMM_LOST: recovering"
            self._enter(Phase.CONNECTING, now)
            self._connect(s, now)
```

Add this public method after `reset()`:

```python
    def internal_error(self, reason: str):
        """A bug in step() (not a bus error): fail the job and latch, so nothing waits on it forever."""
        self.fault = reason
        self._fail_job(reason)
        self._enter(Phase.FAULT, self.clock())
```

Replace `ActuatorThread._run` with:

```python
    def _run(self):
        while not self._stop.wait(self.period):
            with self._lock:
                try:
                    self.master.step()
                except Exception as exc:  # a bug, not a bus error: show it instead of dying silently
                    self.master.internal_error(f"ACTUATOR_INTERNAL: {type(exc).__name__}: {exc}")
```

- [ ] **Step 4: Run the tests.**

Run: `.venv\Scripts\python -m pytest`
Expected: all pass. Check in particular that `test_slave_fault_mid_cycle_latches_until_reset` (MOVE_TIMEOUT is still latched) and `test_reboot_away_from_home_is_latched_for_the_operator` (NOT_HOME is still latched) pass.

- [ ] **Step 5: Update spec §7.** Append this bullet to the "Details settled while building it" list:

```
- A slave COMM_LOST seen while the link is live again (this process stalled > 1 s) is auto-reset the
  same way as at connect. A bug inside the master's step() latches ACTUATOR_INTERNAL instead of
  stopping the thread, and READY needs a status read within the last 500 ms.
```

- [ ] **Step 6: Commit.**

```bash
git add cell/actuator.py tests/test_actuator.py docs/superpowers/specs/2026-10-11-duplo-inspection-design.md
git commit -m "Actuator: latch ACTUATOR_INTERNAL on bugs, require a fresh status, auto-recover a stall's COMM_LOST"
```

---

### Task 6: `inspect_test` finishes the strike on quit and starts the arm after the camera

**Why (PR #1 review, Important 3 and Minors 2 and 5):**
- On Q/ESC, `actuator.stop()` runs at once and the port closes. The slave's watchdog then halts the stick mid-stroke, which is the main way it ends up away from home.
- The actuator thread starts before `start_pipeline()`. A long camera start-up can starve it past the 1 s watchdog.
- After a `join` timeout, the main thread writes the bus while the worker may still be using it.

**Files:**
- Modify: `cell/actuator.py` (`ActuatorThread.stop`, `request_reject`)
- Modify: `tools/inspect_test.py` (`main`)
- Test: `tests/test_actuator.py`

**Interfaces:**
- Consumes: `ActuatorMaster.done_timeout` (Task 3).
- Produces: `ActuatorThread.stop(drain: float = 0.0) -> bool`, which is True when the thread has ended. After `stop()` has started, `ActuatorThread.request_reject()` returns None.

- [ ] **Step 1: Write the failing test.** Append to `tests/test_actuator.py`:

```python
def test_stop_lets_a_running_strike_finish():
    slave = FakeSlave(time.monotonic)
    slave.strike_time = 0.3
    actuator = ActuatorThread(ActuatorMaster(slave), period=0.01).start()
    assert wait_until(lambda: actuator.master.ready)
    job = actuator.request_reject()
    assert actuator.stop(drain=2.0)          # Q pressed right after the strike started
    assert job.state == "done"
    assert slave.coils[COIL_REQ_REJECT] is False
    assert actuator.request_reject() is None  # no new strikes once closing
```

- [ ] **Step 2: Run it and watch it fail.**

Run: `.venv\Scripts\python -m pytest tests/test_actuator.py::test_stop_lets_a_running_strike_finish -v`
Expected: `TypeError: stop() got an unexpected keyword argument 'drain'`.

- [ ] **Step 3: Implement it.** In `ActuatorThread.__init__`, add `self._closing = False`. Replace `stop` and `request_reject`:

```python
    def stop(self, drain: float = 0.0) -> bool:
        """Stop the thread; True once it has ended. drain > 0: first let a running reject cycle
        finish (up to `drain` s), so the slave's watchdog does not halt the stick halfway out."""
        with self._lock:
            self._closing = True
        deadline = time.monotonic() + drain
        while self._thread.is_alive() and self.master.job is not None and time.monotonic() < deadline:
            time.sleep(self.period)
        self._stop.set()
        self._thread.join(timeout=1.0)
        return not self._thread.is_alive()

    def request_reject(self) -> RejectJob | None:
        with self._lock:
            return None if self._closing else self.master.request_reject()
```

- [ ] **Step 4: Change `tools/inspect_test.py`.**
- Move `actuator = ActuatorThread(master).start()` out of the `if not args.no_motor:` block. Initialise `master = None` before the block.
- After `settings.set_auto()`, insert:

```python
    if master is not None:  # start the arm's heartbeat only once the camera is up
        actuator = ActuatorThread(master).start()
```

- Keep `trigger, auto_reject = RejectTrigger(), actuator is not None`, but move it below that insertion.
- Replace the `if actuator:` block in `finally` with:

```python
        if actuator:
            # let a running strike come home before the port closes and the watchdog would halt it
            if actuator.stop(drain=actuator.master.done_timeout):
                try:
                    actuator.master.bus.write_coil(COIL_REQ_REJECT, False)  # leave no request behind
                except BusError:
                    pass
                actuator.master.bus.close()
```

- [ ] **Step 5: Smoke-check that the tool still parses and imports.**

Run: `.venv\Scripts\python tools/inspect_test.py --help`
Expected: the usage text prints and exits 0. That needs `models/meta.json` not to be read before argparse, and it is not: `--help` exits inside `parse_args()`.

- [ ] **Step 6: Run the suite.**

Run: `.venv\Scripts\python -m pytest`
Expected: all pass.

- [ ] **Step 7: Update spec §7.** Append this bullet:

```
- On quit, tools/inspect_test.py stops taking new strikes and waits (up to the DONE timeout) for a
  running cycle to finish before it closes the port. The arm's thread starts only after the camera.
```

- [ ] **Step 8: Commit.**

```bash
git add cell/actuator.py tools/inspect_test.py tests/test_actuator.py docs/superpowers/specs/2026-10-11-duplo-inspection-design.md
git commit -m "inspect_test: finish a running strike before quitting; start the arm after the camera"
```

---

### Task 7: Harden the threshold choice

**Why (PR #2 review, Minors 2–7):**
- NaN scores give an arbitrary threshold. A NaN threshold switches the surface check off.
- The one-sided fallbacks are only printed as a note, so they are easy to miss.
- `separable: False` reads as "overlap" in the one-sided cases.
- One test assertion compares two constants.
- The overlay shows 3 decimals while the overlap margin is 1e-4.
- Spec §10 describes only the two-case rule.

**Files:**
- Modify: `cell/threshold.py`, `train/train_model.py`, `tools/inspect_test.py:70`
- Modify: spec §10
- Test: `tests/test_threshold.py`

**Interfaces:**
- Produces: `choose_threshold` raises `ValueError` on a non-finite score. `meta.json["separable"]` is `True`, `False` or `None`; `None` means "not decided: one side missing".

- [ ] **Step 1: Write the tests.** In `tests/test_threshold.py`:
- Add `import math`.
- Change the import to `from cell.threshold import ANOMALIB_THRESHOLD, MARGIN, choose_threshold`.
- Replace `test_overlap_still_rejects_every_defect` with this version. It already passes and now pins the exact value:

```python
def test_overlap_still_rejects_every_defect():
    assert choose_threshold([0.3, 0.65], [0.55, 0.9]) == (pytest.approx(0.55 - MARGIN), "overlap")
```

- Append:

```python
def test_tie_is_an_overlap():
    assert choose_threshold([0.5], [0.5]) == (pytest.approx(0.5 - MARGIN), "overlap")


def test_single_scores_separate():
    assert choose_threshold([0.2], [0.8]) == (pytest.approx(0.5), "separable")


@pytest.mark.parametrize("good, defect", [([0.3], [math.nan, 0.6]), ([math.nan, 0.3], [0.6]),
                                          ([0.3, math.inf], [0.6]), ([], [math.nan])])
def test_non_finite_scores_are_refused(good, defect):
    with pytest.raises(ValueError, match="non-finite"):
        choose_threshold(good, defect)
```

- [ ] **Step 2: Run them and watch them fail.**

Run: `.venv\Scripts\python -m pytest tests/test_threshold.py -v`
Expected: the 4 `test_non_finite_scores_are_refused` cases fail with "DID NOT RAISE".

- [ ] **Step 3: Implement it in `cell/threshold.py`.** Add `import math` at the top. Make the first lines of `choose_threshold` (after the docstring):

```python
    if not all(math.isfinite(s) for s in (*good, *defect)):
        raise ValueError("non-finite anomaly score in val: the model output is broken, retrain")
```

Add this line to the docstring: `Raises ValueError on a NaN/inf score: any threshold from it would be arbitrary.`

- [ ] **Step 4: Change `train/train_model.py`.** After the `print(f"\nthreshold {threshold:.3f} ({note})")` line, add:

```python
    if basis not in ("separable", "overlap"):
        print(f"WARNING: threshold not validated against both good and defect bricks ({basis}); "
              f"capture more val data before trusting the surface check")
```

In `meta`, change `"separable": basis == "separable",` to `"separable": {"separable": True, "overlap": False}.get(basis),`. It stays None when one side of val is missing.

Also change the docstring line `- if val has no good bricks or no surface defects, see cell/threshold.py.` to `- if val has no good bricks or no surface defects: one-sided, with anomalib's 0.5 as the other bound (cell/threshold.py), and a WARNING.`

- [ ] **Step 5: Show 4 decimals in the overlay.** In `tools/inspect_test.py`'s `inspect()`, change the score line to:

```python
                  f"anomaly score {score:.4f}  (threshold {meta['threshold']:.4f})",
```

- [ ] **Step 6: Update spec §10.** After the sentence `...the value that rejects all val defects, and the overlap is reported.`, add:

```
If val has no surface defects (or no good bricks), the side that exists sets the bound and
anomalib's 0.5 fills in for the other; training prints a WARNING and meta.json records
`threshold_basis`. NaN/inf scores stop training.
```

- [ ] **Step 7: Run the suite.**

Run: `.venv\Scripts\python -m pytest`
Expected: all pass.

- [ ] **Step 8: Commit.**

```bash
git add cell/threshold.py train/train_model.py tools/inspect_test.py tests/test_threshold.py docs/superpowers/specs/2026-10-11-duplo-inspection-design.md
git commit -m "Threshold: refuse NaN scores, warn on one-sided fallbacks, show 4 decimals"
```

---

### Task 8: Hardware bring-up (manual, at the cell)

**Why:** nothing above has run on the Mega, the stepper or a real serial port. This checklist replaces the test plan in PR #1 and covers what Tasks 2–6 changed. Record the result of every line in `docs/findings/2026-10-xx-arm-bringup.md`, using the date of the run.

- [ ] `arduino-cli lib install AccelStepper ModbusRTUSlave`, then compile and upload `firmware/reject_cell`. Note the flash and RAM figures.
- [ ] `.venv\Scripts\python -m pytest`: all green, **0 skipped** (MSVC builds the C++ tests).
- [ ] `python tools/inspect_test.py --port COM4`: the overlay shows `arm: idle`.
- [ ] `K` at the defaults: the stick strikes and returns, and the strike count goes up by 1. **Time the full cycle** (stopwatch or video). Expected about 1.1–1.3 s. Check it is well under the 3.1 s DONE timeout.
- [ ] `python tools/inspect_test.py --speed 400`, then `K`: the cycle completes, with no `MOVE_TIMEOUT` and no `ACTUATOR_TIMEOUT`.
- [ ] `--strike-steps 40000` is refused at start-up with a range message, before the port opens.
- [ ] Auto mode:
  - an NG brick is struck once;
  - a good brick is left alone;
  - a brick that does not move shows `REJECT_FAILED`;
  - **an NG brick with a hand next to it in the box is not struck** (`hand in the plate box: strike held`), and is struck once the hand has gone.
- [ ] Press Q right after a `K`: the stick comes home before the window closes. On restart there is no `NOT_HOME`.
- [ ] Unplug USB in the middle of a slow strike (`--speed 400`), plug it back in and restart the app: the overlay shows `ACTUATOR_FAULT: arm reports NOT_HOME ... press R`. `X` does nothing. Turn the stick home by hand and press `R`: `arm: idle`, and the next strike works.
- [ ] A normal restart of the app with the stick at home: no `NOT_HOME`, and no key needed.
- [ ] Unplug USB while idle: `ACTUATOR_COMM_LOST`. Plug it back in and restart: the arm connects without `X`.
- [ ] Hold the stick during a strike: the stepper is open-loop, so it loses steps and no fault appears; check by eye whether the stick still comes back to the same home, and note it (`MOVE_TIMEOUT` detects a starved `loop()`, not a blocked stick).
- [ ] Edge hand: reach into the plate box from the edge next to an NG brick — `hand in the plate box: strike held`.
- [ ] `--reverse` strikes in the other direction.
- [ ] Optional, a timing check: toggle a spare pin around `modbus.poll()` and measure the per-request pause with a scope or logic analyser. Expected 4–5 ms.
