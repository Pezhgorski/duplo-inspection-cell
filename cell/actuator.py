"""Modbus RTU master for the reject arm (firmware/reject_cell): register map, handshake, watchdogs.

Reject handshake (design spec §7):
    master: READY? -> REQ_REJECT=1        slave: BUSY=1, strike out, return home, BUSY=0, DONE=1
    master: DONE   -> REQ_REJECT=0        slave: DONE=0 -> READY

`ActuatorMaster.step()` never blocks on the handshake: it does one round of bus I/O and moves the
handshake on, so the tests drive it on a simulated clock and `ActuatorThread` runs it beside the
camera loop. The master writes its heartbeat every 100 ms; the slave stops the motor if it goes
quiet for 1 s, and the master faults if the slave's heartbeat stops changing for 500 ms.
"""
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
SLAVE_FAULT_COMM_LOST = 1
SLAVE_FAULTS = {0: "NONE", 1: "COMM_LOST", 2: "MOVE_TIMEOUT"}

HEARTBEAT_PERIOD = 0.1    # s, master heartbeat
HEARTBEAT_TIMEOUT = 0.5   # s, slave heartbeat must change within this
BUSY_TIMEOUT = 0.3        # s, from REQ_REJECT to BUSY
DONE_TIMEOUT = 3.0        # s, from REQ_REJECT to DONE
RELEASE_TIMEOUT = 0.5     # s, from dropping REQ_REJECT to DONE clearing


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
        self.state, self.error = ("failed", error) if error else ("done", "")


class ActuatorMaster:
    def __init__(self, bus, strike_steps: int = 800, speed: int = 2400, accel: int = 12000,
                 clock=time.monotonic):
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

    @property
    def ready(self) -> bool:
        return self.phase is Phase.IDLE and self.job is None and bool(self.status and self.status.ready)

    def request_reject(self) -> RejectJob | None:
        """Start one reject cycle on the next step(); None when the arm cannot take one now."""
        if not self.ready:
            return None
        self.job = RejectJob()
        return self.job

    def reset(self):
        """Operator reset of a latched fault."""
        if self.phase is Phase.FAULT:
            self._reset_requested = True

    def step(self):
        now = self.clock()
        try:
            if now >= self._next_beat:
                self._beat = (self._beat + 1) & 0xFFFF
                self.bus.write_registers(HR_MASTER_HEARTBEAT, [self._beat])
                self._next_beat = now + HEARTBEAT_PERIOD
            status = self.status = self.bus.read_status()
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
            if self._reset_requested:
                self.bus.write_coil(COIL_REQ_REJECT, False)
                self._pulse_reset()
                self._reset_requested = False
                self.fault = ""
                self._enter(Phase.IDLE, now)
        elif s.fault:
            self._latch(f"ACTUATOR_FAULT: arm reports {SLAVE_FAULTS.get(s.fault_code, s.fault_code)}", now)
        elif self.phase is Phase.IDLE:
            if self.job is not None and self.job.state == "pending":
                if not s.ready:
                    self._fail_job("arm not ready")
                    return
                self._expected_count = (s.cycle_count + 1) & 0xFFFF
                self._request_at = now
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
            elif now - self._request_at > DONE_TIMEOUT:
                self._latch(f"ACTUATOR_TIMEOUT: no DONE within {DONE_TIMEOUT} s", now)
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
            self._latch(f"ACTUATOR_FAULT: arm reports {SLAVE_FAULTS.get(s.fault_code, s.fault_code)}", now)
            return
        self.fault = ""
        self._enter(Phase.IDLE, now)

    def _pulse_reset(self):
        # the slave acts on the rising edge; it sees the 1 before the 0 because it updates
        # between two requests
        self.bus.write_coil(COIL_RESET_FAULT, True)
        self.bus.write_coil(COIL_RESET_FAULT, False)

    def _latch(self, reason: str, now):
        self.fault = reason
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
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="actuator", daemon=True)

    def start(self) -> "ActuatorThread":
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def request_reject(self) -> RejectJob | None:
        with self._lock:
            return self.master.request_reject()

    def reset(self):
        with self._lock:
            self.master.reset()

    def describe(self) -> str:
        m = self.master
        count = m.status.cycle_count if m.status else "-"
        return f"arm: {m.phase.value}  strikes {count}" + (f"  | {m.fault}" if m.fault else "")

    def _run(self):
        while not self._stop.wait(self.period):
            with self._lock:
                self.master.step()
