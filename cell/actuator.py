"""Modbus RTU master for the reject arm (firmware/reject_cell): register map, handshake, watchdogs.

Reject handshake (design spec §7):
    master: READY? -> REQ_REJECT=1        slave: BUSY=1, strike out, return home, BUSY=0, DONE=1
    master: DONE   -> REQ_REJECT=0        slave: DONE=0 -> READY

`ActuatorMaster.step()` never blocks on the handshake: it does one round of bus I/O and moves the
handshake on, so the tests drive it on a simulated clock and `ActuatorThread` runs it beside the
camera loop. The master writes its heartbeat every 100 ms; the slave stops the motor if it goes
quiet for 1 s, and the master faults if the slave's heartbeat stops changing for 500 ms. Stroke and DONE timeouts follow the
strike settings (move_timeout()).
"""
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum

SLAVE_ID = 1
BAUD = 115200

# register map: must match firmware/reject_cell/RejectController.h
COIL_REQ_REJECT, COIL_RESET_FAULT = 0, 1
DI_READY, DI_BUSY, DI_DONE, DI_FAULT = 0, 1, 2, 3
HR_MASTER_HEARTBEAT, HR_STRIKE_STEPS, HR_MAX_SPEED, HR_ACCELERATION = 0, 1, 2, 3
IR_SLAVE_HEARTBEAT, IR_FAULT_CODE, IR_CYCLE_COUNT = 0, 1, 2
SLAVE_FAULT_COMM_LOST, SLAVE_FAULT_NOT_HOME = 1, 3
SLAVE_FAULTS = {0: "NONE", 1: "COMM_LOST", 2: "MOVE_TIMEOUT",
                3: "NOT_HOME (restarted mid-strike): put the stick home by hand, press R"}

HEARTBEAT_PERIOD = 0.1    # s, master heartbeat
HEARTBEAT_TIMEOUT = 0.5   # s, slave heartbeat must change within this
BUSY_TIMEOUT = 0.3        # s, from REQ_REJECT to BUSY
RELEASE_TIMEOUT = 0.5     # s, from dropping REQ_REJECT to DONE clearing
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


class BusError(Exception):
    """A Modbus request failed: no reply, bad CRC, or an exception response."""


@dataclass(frozen=True)
class Status:
    ready: bool
    busy: bool
    done: bool
    fault: bool
    heartbeat: int
    fault_code: int
    cycle_count: int


class ModbusBus:
    """The register-level link to the slave, over a pymodbus client."""

    def __init__(self, client, device_id: int = SLAVE_ID):
        self.client = client
        self.device_id = device_id

    @classmethod
    def serial(cls, port: str, timeout: float = 0.1) -> "ModbusBus":
        from pymodbus.client import ModbusSerialClient

        client = ModbusSerialClient(port, baudrate=BAUD, timeout=timeout, retries=0)
        if not client.connect():
            raise BusError(f"cannot open {port}")
        time.sleep(2.0)  # opening the port resets the Mega; let its bootloader hand over
        return cls(client)

    def read_status(self) -> Status:
        bits = self._call(self.client.read_discrete_inputs, DI_READY, count=4).bits
        regs = self._call(self.client.read_input_registers, IR_SLAVE_HEARTBEAT, count=3).registers
        return Status(*(bool(b) for b in bits[:4]), *regs)

    def write_coil(self, address: int, value: bool):
        self._call(self.client.write_coil, address, value)

    def write_registers(self, address: int, values: list[int]):
        self._call(self.client.write_registers, address, [v & 0xFFFF for v in values])

    def close(self):
        self.client.close()

    def _call(self, request, *args, **kwargs):
        from pymodbus.exceptions import ModbusException

        try:
            response = request(*args, device_id=self.device_id, **kwargs)
        except ModbusException as exc:
            raise BusError(str(exc)) from exc
        if response.isError():
            raise BusError(str(response))
        return response


class Phase(Enum):
    CONNECTING = "connecting"      # waiting for a live slave heartbeat
    IDLE = "idle"
    WAIT_BUSY = "wait busy"
    WAIT_DONE = "wait done"
    WAIT_RELEASE = "wait release"
    FAULT = "fault"                # latched: needs reset()


class RejectJob:
    """One reject cycle as seen by the caller: pending -> done | failed."""

    def __init__(self):
        self.state = "pending"
        self.error = ""

    def _finish(self, error: str = ""):
        self.error = error  # before state: another thread may read error as soon as state changes
        self.state = "failed" if error else "done"


class ActuatorMaster:
    def __init__(self, bus, strike_steps: int = 800, speed: int = 2400, accel: int = 12000,
                 clock=time.monotonic):
        check_strike(strike_steps, speed, accel)
        self.done_timeout = 2 * move_timeout(strike_steps, speed, accel) + DONE_MARGIN
        self.bus = bus
        self.config = [strike_steps, speed, accel]
        self.clock = clock
        self.phase = Phase.CONNECTING
        self.fault = ""                 # why the cell cannot reject right now
        self.status: Status | None = None
        self.job: RejectJob | None = None
        now = clock()
        self._beat = 0
        self._next_beat = now
        self._slave_beat = None
        self._slave_beat_at = now
        self._phase_at = now
        self._request_at = now
        self._expected_count = 0
        self._reset_requested = False
        self._latched_code = None       # slave fault code behind a latched fault
        self._status_at = float("-inf")  # when the last status read succeeded

    @property
    def ready(self) -> bool:
        fresh = self.clock() - self._status_at <= HEARTBEAT_TIMEOUT
        return (self.phase is Phase.IDLE and self.job is None and fresh
                and bool(self.status and self.status.ready))

    def request_reject(self) -> RejectJob | None:
        """Start one reject cycle on the next step(); None when the arm cannot take one now."""
        if not self.ready:
            return None
        self.job = RejectJob()
        return self.job

    def reset(self) -> bool:
        """Operator reset of a latched fault (X). NOT_HOME needs confirm_home() (R) instead.
        True when a reset was queued."""
        if self.phase is Phase.FAULT and self._latched_code != SLAVE_FAULT_NOT_HOME:
            self._reset_requested = True
            return True
        return False

    def confirm_home(self) -> bool:
        """Operator (R): the stick is back home by hand, after the arm restarted mid-strike.
        True when a reset was queued."""
        if self.phase is Phase.FAULT and self._latched_code == SLAVE_FAULT_NOT_HOME:
            self._reset_requested = True
            return True
        return False

    def internal_error(self, reason: str):
        """A bug in step() (not a bus error): fail the job and latch, so nothing waits on it forever."""
        self.fault = reason
        self._latched_code = None  # not a slave fault, so X (not R) clears it, even after a NOT_HOME
        self._fail_job(reason)
        self._enter(Phase.FAULT, self.clock())
        try:
            self.bus.write_coil(COIL_REQ_REJECT, False)  # best effort: do not leave a request standing
        except Exception:
            pass

    def step(self):
        now = self.clock()
        try:
            if now >= self._next_beat:
                self._beat = (self._beat + 1) & 0xFFFF
                self.bus.write_registers(HR_MASTER_HEARTBEAT, [self._beat])
                self._next_beat = now + HEARTBEAT_PERIOD
            status = self.status = self.bus.read_status()
            self._status_at = now
        except BusError:
            # one lost reply is not a fault: only a heartbeat that stays stale is
            self._check_alive(now)
            return
        if status.heartbeat != self._slave_beat:
            self._slave_beat, self._slave_beat_at = status.heartbeat, now
        if not self._check_alive(now):
            return
        try:
            self._advance(status, now)
        except BusError:
            pass  # retried on the next step; the timeouts catch a slave that stays silent

    def _check_alive(self, now) -> bool:
        if now - self._slave_beat_at <= HEARTBEAT_TIMEOUT:
            return True
        if self.phase not in (Phase.CONNECTING, Phase.FAULT):
            self._fail_job("ACTUATOR_COMM_LOST")
            self._enter(Phase.CONNECTING, now)
        if self.phase is Phase.CONNECTING:
            self.fault = "ACTUATOR_COMM_LOST: no heartbeat from the arm"
        return False

    def _advance(self, s: Status, now):
        if self.phase is Phase.CONNECTING:
            self._connect(s, now)
        elif self.phase is Phase.FAULT:
            if s.fault and s.fault_code == SLAVE_FAULT_NOT_HOME and self._latched_code != SLAVE_FAULT_NOT_HOME:
                # the arm restarted while we were already faulted (e.g. MOVE_TIMEOUT): X must not
                # take the stick's position as home, so latch NOT_HOME and drop any queued reset
                self._reset_requested = False
                self._latch_slave(s, now)
            elif self._reset_requested:
                self.bus.write_coil(COIL_REQ_REJECT, False)
                # the arm may have rebooted while we were faulted: give RETURNING (and the next
                # strike) the operator's settings, not the firmware defaults
                self.bus.write_registers(HR_STRIKE_STEPS, self.config)
                self._pulse_reset()
                self._reset_requested = False
                self.fault = ""
                self._enter(Phase.IDLE, now)
        elif s.fault and s.fault_code == SLAVE_FAULT_COMM_LOST:
            # the slave lost our heartbeat (this process stalled > 1 s) but the link is fine again:
            # recover exactly as a fresh connect does
            self._fail_job("ACTUATOR_COMM_LOST: the arm lost the master heartbeat")
            self.fault = "ACTUATOR_COMM_LOST: recovering"
            self._enter(Phase.CONNECTING, now)
            self._connect(s, now)
        elif s.fault:
            self._latch_slave(s, now)
        elif self.phase is Phase.IDLE:
            if self.job is not None and self.job.state == "pending":
                if not s.ready:
                    self._fail_job("arm not ready")
                    return
                self._expected_count = (s.cycle_count + 1) & 0xFFFF
                self._request_at = now
                # an unnoticed reboot would leave the firmware defaults in place: re-send the
                # settings before every strike (~5 ms). A failed write leaves the job pending and
                # IDLE, and the next step() retries.
                self.bus.write_registers(HR_STRIKE_STEPS, self.config)
                # enter the wait first: if the write lands but its reply is lost, the cycle
                # still runs, and WAIT_BUSY repeats the (level) request until the slave answers
                self._enter(Phase.WAIT_BUSY, now)
                self.bus.write_coil(COIL_REQ_REJECT, True)
        elif self.phase is Phase.WAIT_BUSY:
            if s.busy or s.done:  # a fast strike can finish between two polls
                self._enter(Phase.WAIT_DONE, now)
            elif now - self._request_at > BUSY_TIMEOUT:
                self._latch(f"ACTUATOR_TIMEOUT: no BUSY within {BUSY_TIMEOUT} s", now)
            else:
                self.bus.write_coil(COIL_REQ_REJECT, True)
        if self.phase is Phase.WAIT_DONE:
            if s.done:
                self._enter(Phase.WAIT_RELEASE, now)
                self.bus.write_coil(COIL_REQ_REJECT, False)
            elif now - self._request_at > self.done_timeout:
                self._latch(f"ACTUATOR_TIMEOUT: no DONE within {self.done_timeout:.1f} s", now)
        elif self.phase is Phase.WAIT_RELEASE:
            if s.done and now - self._phase_at <= RELEASE_TIMEOUT:
                self.bus.write_coil(COIL_REQ_REJECT, False)  # repeat, in case the first was lost
            if not s.done:
                if s.cycle_count != self._expected_count:
                    self._latch(f"ACTUATOR_COUNT_MISMATCH: expected cycle {self._expected_count}, "
                                f"arm reports {s.cycle_count}", now)
                    return
                self.job._finish()
                self.job = None
                self._enter(Phase.IDLE, now)
            elif now - self._phase_at > RELEASE_TIMEOUT:
                self._latch(f"ACTUATOR_TIMEOUT: DONE not cleared within {RELEASE_TIMEOUT} s", now)

    def _connect(self, s: Status, now):
        """Slave heartbeat is live: put the slave in a known state."""
        self.bus.write_coil(COIL_REQ_REJECT, False)
        self.bus.write_registers(HR_STRIKE_STEPS, self.config)
        if s.fault and s.fault_code == SLAVE_FAULT_COMM_LOST:
            # the expected leftover of an earlier master session. The slave takes the reset only
            # once it has seen our heartbeat change, so stay here and retry until the fault clears
            self._pulse_reset()
            return
        if s.fault:
            self._latch_slave(s, now)
            return
        self.fault = ""
        self._enter(Phase.IDLE, now)

    def _pulse_reset(self):
        # the slave acts on the rising edge; it sees the 1 before the 0 because it updates
        # between two requests
        self.bus.write_coil(COIL_RESET_FAULT, True)
        self.bus.write_coil(COIL_RESET_FAULT, False)

    def _latch_slave(self, s: Status, now):
        self._latch(f"ACTUATOR_FAULT: arm reports {SLAVE_FAULTS.get(s.fault_code, s.fault_code)}", now,
                    s.fault_code)

    def _latch(self, reason: str, now, slave_code=None):
        self.fault = reason
        self._latched_code = slave_code
        self._fail_job(reason)
        self._enter(Phase.FAULT, now)
        try:
            self.bus.write_coil(COIL_REQ_REJECT, False)
        except BusError:
            pass

    def _fail_job(self, reason: str):
        if self.job is not None:
            self.job._finish(reason)
            self.job = None

    def _enter(self, phase: Phase, now):
        self.phase, self._phase_at = phase, now


class ActuatorThread:
    """Runs ActuatorMaster.step() every `period` s in the background, so the camera loop never
    waits on the bus. Each Modbus request pauses the slave's stepping for ~5 ms, so keep this
    near 50 ms rather than polling as fast as possible."""

    def __init__(self, master: ActuatorMaster, period: float = 0.05):
        self.master = master
        self.period = period
        self._lock = threading.Lock()
        self._closing = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="actuator", daemon=True)

    def start(self) -> "ActuatorThread":
        self._thread.start()
        return self

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

    def reset(self) -> bool:
        with self._lock:
            return self.master.reset()

    def confirm_home(self) -> bool:
        with self._lock:
            return self.master.confirm_home()

    def describe(self) -> str:
        m = self.master
        count = m.status.cycle_count if m.status else "-"
        return f"arm: {m.phase.value}  strikes {count}" + (f"  | {m.fault}" if m.fault else "")

    def _run(self):
        while not self._stop.wait(self.period):
            with self._lock:
                try:
                    self.master.step()
                except Exception as exc:  # a bug, not a bus error: show it instead of dying silently
                    self.master.internal_error(f"ACTUATOR_INTERNAL: {type(exc).__name__}: {exc}")
