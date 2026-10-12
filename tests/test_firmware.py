"""Slave side: the firmware logic (firmware/reject_cell/RejectController) built for the PC.

1. The C++ unit tests in tests/firmware/reject_controller_test.cpp.
2. The Python master against the real slave logic, loaded with ctypes: proves both ends
   agree on the register map and the handshake.

Needs a C++ compiler: g++ / clang++ on PATH, or Visual Studio C++ build tools (found with vswhere); skipped without one.
"""
import ctypes
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from cell.actuator import (COIL_REQ_REJECT, DI_BUSY, DI_DONE, DI_FAULT, DI_READY, HR_ACCELERATION,
                           HR_MASTER_HEARTBEAT, HR_MAX_SPEED, HR_STRIKE_STEPS, IR_CYCLE_COUNT,
                           IR_FAULT_CODE, IR_SLAVE_HEARTBEAT, ActuatorMaster, Phase, Status, move_timeout)
from sim import SimClock, run

ROOT = Path(__file__).resolve().parent.parent
FIRMWARE = ROOT / "firmware" / "reject_cell" / "RejectController.cpp"
TESTS = ROOT / "tests" / "firmware"


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


def test_controller_unit_tests(tmp_path):
    exe = tmp_path / "reject_controller_test"
    compile_cpp([TESTS / "reject_controller_test.cpp", FIRMWARE], exe)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.stdout


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    suffix = {"win32": ".dll", "darwin": ".dylib"}.get(sys.platform, ".so")
    path = tmp_path_factory.mktemp("sim") / f"slave_sim{suffix}"
    compile_cpp([TESTS / "slave_sim.cpp", FIRMWARE], path, shared=True)
    lib = ctypes.CDLL(str(path))
    lib.sim_new.restype = ctypes.c_void_p
    for name in ("sim_free", "sim_advance", "sim_update", "sim_coil", "sim_set_coil", "sim_discrete",
                 "sim_holding", "sim_set_holding", "sim_input", "sim_position", "sim_max_position",
                 "sim_min_position", "sim_set_stalled", "sim_state", "sim_reboot", "sim_away"):
        getattr(lib, name).argtypes = [ctypes.c_void_p] + [ctypes.c_int] * {
            "sim_advance": 1, "sim_coil": 1, "sim_set_coil": 2, "sim_discrete": 1, "sim_holding": 1,
            "sim_set_holding": 2, "sim_input": 1, "sim_set_stalled": 1}.get(name, 0)
    for name in ("sim_position", "sim_max_position", "sim_min_position"):
        getattr(lib, name).restype = ctypes.c_long
    lib.sim_stroke_timeout_ms.argtypes = [ctypes.c_int] * 3
    lib.sim_stroke_timeout_ms.restype = ctypes.c_uint32
    return lib


class SimSlave:
    """The firmware logic behind the master's bus interface. After each write the slave runs one
    loop() pass, as on the Mega, where modbus.poll() answers and controller.update() follows."""

    def __init__(self, lib):
        self.lib = lib
        self.sim = lib.sim_new()

    def read_status(self) -> Status:
        d = lambda a: bool(self.lib.sim_discrete(self.sim, a))
        i = lambda a: self.lib.sim_input(self.sim, a)
        return Status(d(DI_READY), d(DI_BUSY), d(DI_DONE), d(DI_FAULT),
                      i(IR_SLAVE_HEARTBEAT), i(IR_FAULT_CODE), i(IR_CYCLE_COUNT))

    def write_coil(self, address, value):
        self.lib.sim_set_coil(self.sim, address, int(value))
        self.lib.sim_update(self.sim)

    def write_registers(self, address, values):
        for offset, value in enumerate(values):
            self.lib.sim_set_holding(self.sim, address + offset, value & 0xFFFF)
        self.lib.sim_update(self.sim)

    def advance(self, seconds):
        self.lib.sim_advance(self.sim, round(seconds * 1000))

    def __getattr__(self, name):  # sim_position(...) -> slave.position()
        fn = getattr(self.lib, f"sim_{name}")
        return lambda *args: fn(self.sim, *args)

    def free(self):
        self.lib.sim_free(self.sim)


@pytest.fixture
def clock():
    return SimClock()


@pytest.fixture
def slave(lib):
    s = SimSlave(lib)
    yield s
    s.free()


def connect(slave, clock, **config):
    m = ActuatorMaster(slave, clock=clock, **config)
    assert run(m, clock, 1.0, slave_advance=slave.advance, until=lambda: m.ready)
    return m


def reject(master, slave, clock, seconds=3.0):
    job = master.request_reject()
    assert job is not None
    run(master, clock, seconds, slave_advance=slave.advance, until=lambda: job.state != "pending")
    return job


def test_handshake_with_real_firmware_logic(slave, clock):
    m = connect(slave, clock)
    assert slave.holding(HR_STRIKE_STEPS) == 800
    assert slave.holding(HR_MAX_SPEED) == 2400
    assert slave.holding(HR_ACCELERATION) == 12000
    assert slave.holding(HR_MASTER_HEARTBEAT) > 0
    job = reject(m, slave, clock)
    assert job.state == "done", m.fault
    assert slave.max_position() == 800
    assert slave.position() == 0
    assert slave.input(IR_CYCLE_COUNT) == 1
    assert slave.coil(COIL_REQ_REJECT) == 0
    run(m, clock, 0.1, slave_advance=slave.advance)
    assert m.ready and slave.state() == IDLE


def test_repeated_rejects(slave, clock):
    m = connect(slave, clock)
    for _ in range(3):
        assert reject(m, slave, clock).state == "done", m.fault
        run(m, clock, 0.1, slave_advance=slave.advance)
    assert slave.input(IR_CYCLE_COUNT) == 3


def test_reverse_strike_and_speed_from_master(slave, clock):
    m = connect(slave, clock, strike_steps=-600, speed=1200)
    assert reject(m, slave, clock).state == "done", m.fault
    assert slave.min_position() == -600
    assert slave.max_position() == 0


def test_stalled_arm_reported_and_recovered(slave, clock):
    m = connect(slave, clock)
    slave.set_stalled(1)
    job = reject(m, slave, clock)
    assert job.state == "failed"
    assert m.phase is Phase.FAULT
    assert m.fault == "ACTUATOR_FAULT: arm reports MOVE_TIMEOUT"
    assert slave.state() == FAULTED
    # operator frees the arm and resets: it returns home, then takes the next reject
    slave.set_stalled(0)
    m.reset()
    assert run(m, clock, 1.0, slave_advance=slave.advance, until=lambda: m.ready)
    assert slave.position() == 0
    assert reject(m, slave, clock).state == "done", m.fault


def test_master_freeze_mid_strike_stops_the_arm(slave, clock):
    m = connect(slave, clock, speed=400)  # 800 steps out take 2 s
    job = m.request_reject()
    run(m, clock, 0.3, slave_advance=slave.advance)
    # the vision PC hangs for 1.5 s: no heartbeat reaches the slave
    slave.advance(1.5)
    clock.advance(1.5)
    assert slave.state() == FAULTED
    stopped = slave.position()
    assert 0 < stopped < 800
    slave.advance(1.0)
    assert slave.position() == stopped  # halted, not still moving
    run(m, clock, 0.2, slave_advance=slave.advance)
    assert job.state == "failed" and "COMM_LOST" in job.error
    # the link is fine again: the master resets the watchdog fault itself and the arm drives home
    assert m.phase is not Phase.FAULT
    assert run(m, clock, 3.0, slave_advance=slave.advance, until=lambda: m.ready)
    assert slave.position() == 0


def test_restarted_master_clears_the_watchdog_fault(slave, clock):
    first = connect(slave, clock)
    del first  # vision app closed; the slave's watchdog fires
    slave.advance(2.0)
    clock.advance(2.0)
    assert slave.state() == FAULTED
    second = connect(slave, clock)
    assert second.fault == ""
    assert slave.input(IR_CYCLE_COUNT) == 0  # no strike on reconnect
    assert reject(second, slave, clock).state == "done"


@pytest.mark.parametrize("steps, speed, accel", [(800, 2400, 12000), (-600, 1200, 12000),
                                                 (800, 300, 12000), (100, 4000, 2000)])
def test_master_and_slave_agree_on_the_stroke_timeout(lib, steps, speed, accel):
    assert lib.sim_stroke_timeout_ms(steps, speed, accel) == pytest.approx(
        move_timeout(steps, speed, accel) * 1000, abs=2)


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


def test_reboot_while_already_faulted_still_needs_r(slave, clock):
    m = connect(slave, clock, speed=400)
    m.request_reject()
    run(m, clock, 0.5, slave_advance=slave.advance)
    slave.set_stalled(1)
    run(m, clock, 6.0, slave_advance=slave.advance)
    assert m.phase is Phase.FAULT and "MOVE_TIMEOUT" in m.fault
    assert slave.position() > 0 and slave.away()
    slave.reboot()  # the Mega restarts while the master is already faulted
    run(m, clock, 0.3, slave_advance=slave.advance)
    assert m.phase is Phase.FAULT and "NOT_HOME" in m.fault
    m.reset()  # X must not take the stick's position as home
    run(m, clock, 0.5, slave_advance=slave.advance)
    assert m.phase is Phase.FAULT and slave.away()
    slave.set_stalled(0)
    m.confirm_home()  # R
    assert run(m, clock, 1.0, slave_advance=slave.advance, until=lambda: m.ready)
    assert not slave.away()
