"""Master side: the handshake, timeouts and comm watchdog against a scriptable fake slave.

The fake behaves like the firmware unless a test tells it to misbehave; the real firmware logic
is exercised against this master in test_firmware.py.
"""
import time

import pytest

from cell.actuator import (BUSY_TIMEOUT, COIL_REQ_REJECT, COIL_RESET_FAULT, DONE_TIMEOUT,
                           HR_MASTER_HEARTBEAT, HR_STRIKE_STEPS, ActuatorMaster, ActuatorThread,
                           BusError, Phase, Status)
from sim import SimClock, run


class FakeSlave:
    """Register-level slave on the shared clock, used directly as the master's bus."""

    def __init__(self, clock):
        self.clock = clock
        self.coils = [False, False]
        self.holding = [0, 0, 0, 0]
        self.ready, self.busy, self.done, self.fault = True, False, False, False
        self.fault_code = 0
        self.count = 0
        self.heartbeat = 0
        self.strikes = 0
        self.resets = 0
        self.coil_writes = []
        # misbehaviour knobs
        self.strike_time = 0.4
        self.beating = True
        self.offline = False
        self.ignore_requests = False
        self.never_release = False
        self.count_step = 1
        self.lose_reply = None  # (address, value): apply that coil write once, then drop its reply
        self._strike_end = None

    def read_status(self) -> Status:
        self._io()
        return Status(self.ready, self.busy, self.done, self.fault, self.heartbeat, self.fault_code, self.count)

    def write_coil(self, address, value):
        self._io()
        self.coil_writes.append((address, value))
        rising = value and not self.coils[address]
        self.coils[address] = value
        if address == COIL_REQ_REJECT and rising and self.ready and not self.ignore_requests:
            self.ready, self.busy = False, True
            self._strike_end = self.clock() + self.strike_time
            self.strikes += 1
        if address == COIL_RESET_FAULT and rising and self.fault:
            self.fault, self.fault_code, self.ready = False, 0, True
            self.resets += 1
        if self.lose_reply == (address, value):
            self.lose_reply = None
            raise BusError("reply lost")

    def write_registers(self, address, values):
        self._io()
        self.holding[address:address + len(values)] = values

    def trip(self, code):
        """The slave faults on its own (watchdog, move timeout)."""
        self.fault, self.fault_code = True, code
        self.ready = self.busy = self.done = False
        self._strike_end = None

    def _io(self):
        if self.offline:
            raise BusError("no reply")
        now = self.clock()
        if self.beating:
            self.heartbeat = int(now * 10) & 0xFFFF
        if self._strike_end is not None and now >= self._strike_end:
            self.busy, self.done, self._strike_end = False, True, None
            self.count += self.count_step
        if self.done and not self.coils[COIL_REQ_REJECT] and not self.never_release:
            self.done, self.ready = False, True


@pytest.fixture
def clock():
    return SimClock()


@pytest.fixture
def slave(clock):
    return FakeSlave(clock)


@pytest.fixture
def master(slave, clock):
    m = ActuatorMaster(slave, strike_steps=800, speed=2400, accel=12000, clock=clock)
    run(m, clock, 0.3)
    return m


def reject(master, clock, seconds=2.0):
    job = master.request_reject()
    assert job is not None
    run(master, clock, seconds, until=lambda: job.state != "pending")
    return job


def test_connects_and_configures_the_slave(master, slave):
    assert master.phase is Phase.IDLE
    assert master.ready
    assert master.fault == ""
    assert slave.holding[HR_STRIKE_STEPS:] == [800, 2400, 12000]
    assert (COIL_REQ_REJECT, False) in slave.coil_writes  # starts from a known state


def test_waits_for_a_live_heartbeat_before_connecting(clock, slave):
    slave.beating = False
    m = ActuatorMaster(slave, clock=clock)
    run(m, clock, 2.0)
    assert m.phase is Phase.CONNECTING
    assert "COMM_LOST" in m.fault
    assert m.request_reject() is None
    slave.beating = True
    run(m, clock, 0.2)
    assert m.phase is Phase.IDLE and m.fault == ""


def test_writes_heartbeat_every_100ms(master, slave, clock):
    start = slave.holding[HR_MASTER_HEARTBEAT]
    run(master, clock, 1.0)
    assert slave.holding[HR_MASTER_HEARTBEAT] - start == 10


def test_heartbeat_wraps_at_16_bits(master, slave, clock):
    master._beat = 0xFFFF
    run(master, clock, 0.1)
    assert slave.holding[HR_MASTER_HEARTBEAT] == 0


def test_full_handshake(master, slave, clock):
    job = reject(master, clock)
    assert job.state == "done"
    assert slave.strikes == 1
    assert slave.coils[COIL_REQ_REJECT] is False  # dropped after DONE
    assert master.phase is Phase.IDLE
    run(master, clock, 0.2)
    assert master.ready


def test_fast_strike_between_two_polls(master, slave, clock):
    slave.strike_time = 0.01  # BUSY is never seen, only DONE
    assert reject(master, clock).state == "done"


def test_lost_reply_to_the_request(master, slave, clock):
    slave.lose_reply = (COIL_REQ_REJECT, True)  # the arm got REQ, the master never heard back
    job = reject(master, clock)
    assert job.state == "done", master.fault
    assert slave.strikes == 1
    assert slave.coils[COIL_REQ_REJECT] is False
    run(master, clock, 0.1)
    assert master.ready


def test_lost_reply_to_the_release(master, slave, clock):
    slave.lose_reply = (COIL_REQ_REJECT, False)
    job = reject(master, clock)
    assert job.state == "done", master.fault
    run(master, clock, 0.1)
    assert master.ready


def test_request_lost_on_the_wire_is_resent(master, slave, clock):
    real = slave.write_coil
    calls = []

    def flaky(address, value):  # the first REQ never reaches the slave
        calls.append((address, value))
        if (address, value) == (COIL_REQ_REJECT, True) and calls.count((address, value)) == 1:
            raise BusError("no reply")
        real(address, value)

    slave.write_coil = flaky
    assert reject(master, clock).state == "done", master.fault
    assert slave.strikes == 1


def test_one_request_at_a_time(master, clock):
    job = master.request_reject()
    assert job is not None
    assert master.request_reject() is None
    run(master, clock, 0.1)
    assert master.request_reject() is None  # mid-cycle


def test_consecutive_rejects(master, slave, clock):
    for _ in range(3):
        assert reject(master, clock).state == "done"
        run(master, clock, 0.1)
    assert slave.strikes == 3 and slave.count == 3


def test_no_busy_times_out(master, slave, clock):
    slave.ignore_requests = True
    job = reject(master, clock)
    assert job.state == "failed"
    assert master.phase is Phase.FAULT
    assert master.fault.startswith("ACTUATOR_TIMEOUT") and "BUSY" in master.fault
    assert slave.coils[COIL_REQ_REJECT] is False
    assert clock() - 100.3 == pytest.approx(BUSY_TIMEOUT + 0.05, abs=0.06)


def test_no_done_times_out(master, slave, clock):
    slave.strike_time = 10.0
    job = reject(master, clock, seconds=5.0)
    assert job.state == "failed"
    assert "DONE" in master.fault
    assert clock() - 100.3 == pytest.approx(DONE_TIMEOUT + 0.05, abs=0.06)


def test_done_never_cleared_times_out(master, slave, clock):
    slave.never_release = True
    job = reject(master, clock)
    assert job.state == "failed"
    assert "not cleared" in master.fault


def test_cycle_count_mismatch_is_a_fault(master, slave, clock):
    slave.count_step = 2  # e.g. the arm struck twice for one request
    job = reject(master, clock)
    assert job.state == "failed"
    assert master.fault.startswith("ACTUATOR_COUNT_MISMATCH")


def test_slave_fault_mid_cycle_latches_until_reset(master, slave, clock):
    job = master.request_reject()
    run(master, clock, 0.1)
    slave.trip(2)
    run(master, clock, 0.1)
    assert job.state == "failed"
    assert master.phase is Phase.FAULT
    assert master.fault == "ACTUATOR_FAULT: arm reports MOVE_TIMEOUT"
    run(master, clock, 2.0)
    assert master.phase is Phase.FAULT  # latched
    master.reset()
    run(master, clock, 0.1)
    assert slave.resets == 1
    assert master.phase is Phase.IDLE and master.fault == ""
    assert reject(master, clock).state == "done"


def test_master_fault_reset_without_slave_fault(master, slave, clock):
    slave.ignore_requests = True
    reject(master, clock)
    slave.ignore_requests = False
    master.reset()
    run(master, clock, 0.1)
    assert master.ready
    assert slave.resets == 0  # the reset pulse only clears a slave that is faulted
    assert reject(master, clock).state == "done"


def test_reset_ignored_when_not_faulted(master, slave, clock):
    master.reset()
    run(master, clock, 0.2)
    assert (COIL_RESET_FAULT, True) not in slave.coil_writes


def test_stale_slave_heartbeat_is_comm_lost(master, slave, clock):
    slave.strike_time = 2.0  # the link drops mid-strike
    job = master.request_reject()
    run(master, clock, 0.1)
    slave.beating = False
    run(master, clock, 0.45)
    assert master.phase is not Phase.CONNECTING  # within the 0.5 s grace
    run(master, clock, 0.2)
    assert master.phase is Phase.CONNECTING
    assert master.fault.startswith("ACTUATOR_COMM_LOST")
    assert job.state == "failed"
    slave.beating = True
    run(master, clock, 0.2)
    assert master.phase is Phase.IDLE and master.fault == ""


def test_single_lost_reply_is_not_a_fault(master, slave, clock):
    slave.offline = True
    run(master, clock, 0.2)
    slave.offline = False
    run(master, clock, 0.2)
    assert master.phase is Phase.IDLE and master.fault == ""


def test_lost_link_recovers_and_clears_slave_comm_fault(master, slave, clock):
    slave.offline = True
    run(master, clock, 1.5)
    assert master.phase is Phase.CONNECTING
    slave.trip(1)  # meanwhile the slave's watchdog fired
    slave.offline = False
    run(master, clock, 0.2)
    assert slave.resets == 1  # COMM_LOST is the expected leftover: reset automatically
    assert master.phase is Phase.IDLE
    assert master.ready


def test_other_slave_fault_at_connect_needs_the_operator(clock, slave):
    slave.trip(2)
    m = ActuatorMaster(slave, clock=clock)
    run(m, clock, 0.3)
    assert m.phase is Phase.FAULT
    assert "MOVE_TIMEOUT" in m.fault
    assert slave.resets == 0


def test_thread_runs_the_handshake_in_real_time():
    slave = FakeSlave(time.monotonic)
    slave.strike_time = 0.1
    actuator = ActuatorThread(ActuatorMaster(slave), period=0.01).start()
    try:
        deadline = time.monotonic() + 2.0
        while not actuator.master.ready and time.monotonic() < deadline:
            time.sleep(0.01)
        job = actuator.request_reject()
        assert job is not None
        while job.state == "pending" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert job.state == "done"
        assert "idle" in actuator.describe()
    finally:
        actuator.stop()
