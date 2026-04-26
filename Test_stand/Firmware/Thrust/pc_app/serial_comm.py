####################################################################
# File Name          : serial_comm.py
# Author             : Tinta T.
# Version            : V1.0.0
# Date               : 2026/02/09
# Description        : Serial port manager and UART frame reader
####################################################################

from __future__ import annotations

import time
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

from .logger import DataLogger
from .protocol import SIG_SOF


class SerialManager:
    def __init__(self) -> None:
        self._serial: Optional["serial.Serial"] = None
        self._logger = DataLogger()


    # Pyserial availability check
    @property
    def available(self) -> bool:
        return serial is not None and list_ports is not None
    

    # Return device names for all visible serial ports
    def list_ports(self) -> list[str]: 
        if list_ports is None:
            return []
        return [port.device for port in list_ports.comports()]
    

    # Open a new serial connection (close any existing)
    def connect(self, port: str, baud_rate: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed")
        self.disconnect()
        self._serial = serial.Serial(
            port,
            baudrate=baud_rate,
            timeout=0.3,
            write_timeout=0.5,
        )


    # Close serial port if open
    def disconnect(self) -> None:
        if self._serial:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001
                pass
        self._serial = None


    # True if serial instance exists and is open
    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open
    

    # Verify the port is still open; disconnect on errors
    def check_connection(self) -> bool:
        if not self._serial:
            return False
        try:
            _ = self._serial.in_waiting
            return self._serial.is_open
        except Exception:  # noqa: BLE001
            self.disconnect()
            return False
        

    # Clear buffered RX bytes
    def reset_input(self) -> None:
        if self._serial:
            self._serial.reset_input_buffer()


    # Write raw bytes to the port
    def write(self, data: bytes) -> None:
        if not self._serial:
            raise RuntimeError("Serial port not open")
        self._serial.write(data)


    # Read a single framed packet using SIG_SOF + LEN convention
    def read_frame(
        self,
        timeout_s: float = 1.0,
        buf: bytearray | None = None,
    ) -> bytes | None:
        if not self._serial:
            return None
        deadline = time.monotonic() + timeout_s

        if buf is None:
            buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._serial.read(1)
            
            if chunk:
                buf.extend(chunk)
            else:
                continue
            while len(buf) >= 2:
                if buf[0] != SIG_SOF:
                    buf.pop(0)
                    continue
                length = buf[1]
                total_len = 2 + length
                if len(buf) < total_len:
                    break
                frame = bytes(buf[:total_len])
                del buf[:total_len]
                self._logger.log_frame("uart-rx", frame)
                return frame
        return None


    # Read raw UART bytes until there is no new data for idle_timeout_s.
    def read_raw_until_idle(
        self,
        idle_timeout_s: float = 1.0,
        max_duration_s: float = 60.0,
    ) -> bytes:
        if not self._serial:
            return b""
        start = time.monotonic()
        last_rx = start
        out = bytearray()
        while (time.monotonic() - start) < max_duration_s:
            waiting = 0
            try:
                waiting = int(self._serial.in_waiting)
            except Exception:  # noqa: BLE001
                break

            if waiting > 0:
                chunk = self._serial.read(waiting)
                if chunk:
                    out.extend(chunk)
                    last_rx = time.monotonic()
                continue

            if out and (time.monotonic() - last_rx) >= idle_timeout_s:
                break
            time.sleep(0.01)
        return bytes(out)
