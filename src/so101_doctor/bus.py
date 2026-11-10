"""A read-only Feetech STS bus: ping and read, nothing else.

This module has no way to write a register. The only instructions it can send are PING (0x01) and READ (0x02), so
running so101-doctor cannot move your arm, change a setting or turn torque on or off.
"""
from __future__ import annotations

import time

PING, READ = 0x01, 0x02
ALLOWED = (PING, READ)  # the only instructions this module can send (tests check it)
BAUD = 1_000_000  # SO-101 / LeRobot default
ID_ADDR = 5  # the ID register
WCH = 0x1A86  # USB vendor of the CH34x bridge on the usual SO-101 servo board


class BusError(Exception):
    pass


def packet(servo_id: int, inst: int, params: bytes = b"") -> bytes:
    if inst not in ALLOWED:
        raise ValueError(f"instruction 0x{inst:02x} refused: so101-doctor only pings and reads")
    body = bytes([servo_id, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])


class Bus:
    """Ping and read STS servos over a serial port (pyserial), with retries."""

    def __init__(self, port: str, baud: int = BAUD, timeout: float = 0.05, retries: int = 3):
        import serial  # imported here so --demo works without a serial stack

        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as e:
            raise BusError(f"cannot open {port}: {e}. Check the name (so101-doctor --list-ports); if it is right, "
                           "close any other program using the port (LeRobot, a teleop script).") from e
        self.timeout, self.retries = timeout, retries

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _transact(self, servo_id: int, inst: int, params: bytes, n_data: int):
        """Send one packet, return (error byte, data) of the servo's reply, or None on timeout/garbage."""
        tx = packet(servo_id, inst, params)
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        self.ser.flush()
        buf, deadline = b"", time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            got = self.ser.read(256)
            if not got:
                time.sleep(0.001)
                continue
            buf += got
            r = reply(buf, tx, servo_id, n_data)
            if r is not None:
                return r
        return None

    def ping(self, servo_id: int) -> bool:
        """True if a servo answers. A ping reply whose only error bit is voltage is byte for byte the ping itself,
        and so is an adapter's echo of it; a 1-byte read of the ID register (reply and request differ in length)
        settles that case."""
        for _ in range(self.retries):
            if self._transact(servo_id, PING, b"", 0) is not None:
                return True
        return self._transact(servo_id, READ, bytes([ID_ADDR, 1]), 1) is not None

    def read(self, servo_id: int, addr: int, n: int):
        """Read n bytes from addr. Returns (error byte, bytes)."""
        for _ in range(self.retries):
            r = self._transact(servo_id, READ, bytes([addr, n]), n)
            if r is not None:
                return r
        raise BusError(f"servo {servo_id}: no valid reply reading {n} bytes at {addr} after {self.retries} tries")


def reply(buf: bytes, tx: bytes, servo_id: int, n_data: int):
    """(error byte, data) of the first valid reply frame in buf (ff ff id len err data... chk), or None so far.
    Some adapters echo what they send, and a ping (or a 2-byte read) has the same shape as its reply, so one frame
    equal to tx is taken as the echo wherever it sits in buf; a second one is the reply."""
    want, k, echo = 6 + n_data, 0, False
    while (k := buf.find(b"\xff\xff", k)) >= 0 and len(buf) - k >= want:
        p = buf[k:k + want]
        if p[2] == servo_id and p[3] == n_data + 2 and p[-1] == (~sum(p[2:-1]) & 0xFF):
            if p != tx or echo:
                return p[4], p[5:-1]
            echo, k = True, k + want
        else:
            k += 1  # not our reply: look for the next header
    return None


def usb_ports():
    """Serial ports that are USB devices, the bus servo adapter first (CH343, 1a86:55d3, the usual SO-101 board)."""
    from serial.tools import list_ports

    ports = [p for p in list_ports.comports() if p.vid is not None]
    ports.sort(key=lambda p: (p.vid, p.pid) != (WCH, 0x55D3))
    return ports
