"""ModbusBus over a real pymodbus client against a minimal RTU slave written here: checks the
function codes, addresses, frame layout and CRC the master puts on the wire.

The slave speaks RTU frames over TCP instead of a COM port; the bytes are the same.
"""
import socket
import struct
import threading

import pytest

pytest.importorskip("pymodbus")
from pymodbus import FramerType  # noqa: E402
from pymodbus.client import ModbusTcpClient  # noqa: E402

from cell.actuator import (COIL_REQ_REJECT, COIL_RESET_FAULT, DI_READY, HR_MASTER_HEARTBEAT,  # noqa: E402
                           HR_STRIKE_STEPS, IR_SLAVE_HEARTBEAT, SLAVE_ID, BusError, ModbusBus,
                           Status)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(body: bytes) -> bytes:
    return body + struct.pack("<H", crc16(body))


class RtuSlave:
    """Serves coils / discrete inputs / holding / input registers as plain lists, one client."""

    def __init__(self, unit=SLAVE_ID):
        self.unit = unit
        self.coils, self.discrete = [False] * 8, [False] * 8
        self.holding, self.input = [0] * 8, [0] * 8
        self.requests = []  # (function code, address) as received
        self.sock = socket.create_server(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        conn, _ = self.sock.accept()
        buf = b""
        with conn:
            while True:
                data = conn.recv(256)
                if not data:
                    return
                buf += data
                while len(buf) >= 8:
                    size = 9 + buf[6] if buf[1] in (15, 16) else 8
                    if len(buf) < size:
                        break
                    request, buf = buf[:size], buf[size:]
                    assert struct.unpack("<H", request[-2:])[0] == crc16(request[:-2]), "bad CRC"
                    reply = self._handle(request[:-2])
                    if reply is not None:
                        conn.sendall(frame(reply))

    def _handle(self, req: bytes):
        unit, fc = req[0], req[1]
        if unit != self.unit:
            return None  # not addressed to us: stay silent, as on a real bus
        address, value = struct.unpack(">HH", req[2:6])
        self.requests.append((fc, address))
        head = bytes([unit, fc])
        if fc in (1, 2):
            bits = (self.coils if fc == 1 else self.discrete)[address:address + value]
            packed = bytearray((value + 7) // 8)
            for i, bit in enumerate(bits):
                packed[i // 8] |= bit << (i % 8)
            return head + bytes([len(packed)]) + bytes(packed)
        if fc in (3, 4):
            regs = (self.holding if fc == 3 else self.input)[address:address + value]
            return head + bytes([2 * value]) + struct.pack(f">{value}H", *regs)
        if fc == 5:
            self.coils[address] = value == 0xFF00
            return req
        if fc == 6:
            self.holding[address] = value
            return req
        if fc == 16:
            self.holding[address:address + value] = struct.unpack(f">{value}H", req[7:7 + 2 * value])
            return req[:6]
        if fc == 15:
            for i in range(value):
                self.coils[address + i] = bool(req[7 + i // 8] >> (i % 8) & 1)
            return req[:6]
        return head[:1] + bytes([fc | 0x80, 1])  # illegal function


@pytest.fixture
def slave():
    s = RtuSlave()
    yield s
    s.sock.close()


def make_bus(slave, device_id=SLAVE_ID, timeout=1.0):
    client = ModbusTcpClient("127.0.0.1", port=slave.port, framer=FramerType.RTU, timeout=timeout, retries=0)
    assert client.connect()
    return ModbusBus(client, device_id=device_id)


def test_read_status_maps_bits_and_registers(slave):
    slave.discrete[:4] = [True, False, True, False]  # READY, DONE
    slave.input[:3] = [1234, 2, 7]
    bus = make_bus(slave)
    assert bus.read_status() == Status(ready=True, busy=False, done=True, fault=False,
                                       heartbeat=1234, fault_code=2, cycle_count=7)
    assert slave.requests == [(2, DI_READY), (4, IR_SLAVE_HEARTBEAT)]
    bus.close()


def test_write_coil(slave):
    bus = make_bus(slave)
    bus.write_coil(COIL_RESET_FAULT, True)
    assert slave.coils[:2] == [False, True]
    bus.write_coil(COIL_RESET_FAULT, False)
    bus.write_coil(COIL_REQ_REJECT, True)
    assert slave.coils[:2] == [True, False]
    assert {fc for fc, _ in slave.requests} == {5}
    bus.close()


def test_write_registers_including_negative_steps(slave):
    bus = make_bus(slave)
    bus.write_registers(HR_MASTER_HEARTBEAT, [42])
    bus.write_registers(HR_STRIKE_STEPS, [-600, 2400, 12000])
    assert slave.holding[:4] == [42, 0x10000 - 600, 2400, 12000]
    bus.close()


def test_silent_slave_is_a_bus_error(slave):
    bus = make_bus(slave, device_id=SLAVE_ID + 1, timeout=0.3)
    with pytest.raises(BusError):
        bus.read_status()
    bus.close()
