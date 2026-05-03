r"""
Send a single EDF power percent command to the drone (same framing as the Jupyter scripts).

Usage examples (PowerShell):
  .\.venv\Scripts\python.exe .\edf_set_power.py --port COM5 --baud 9600 --power 30
  .\.venv\Scripts\python.exe .\edf_set_power.py --port COM5 --baud 9600
    (interactive: type 0..100, or 'q' to quit)
"""

from __future__ import annotations

import argparse
import struct
import time

import serial

from protocol import FLAG_DATA, ID_DRONE, ID_PC, PROTOCOL_VER, build_frame


OPT_CAL_PARAM = 0x60  # same as notebook


def encode_servo_angle(angle_deg: int) -> int:
    # Drone expects stored_angle = real_angle + 90
    encoded = int(angle_deg) + 90
    if encoded < -128 or encoded > 127:
        raise ValueError(f"Encoded servo value out of int8 range: {encoded}")
    return encoded


def build_calib_frame(
    edf_pwr_percent: int,
    *,
    x_plus_angle_deg: int = 0,
    x_minus_angle_deg: int = 0,
    y_plus_angle_deg: int = 0,
    y_minus_angle_deg: int = 0,
) -> bytes:
    pwr = int(edf_pwr_percent)
    if not (0 <= pwr <= 100):
        raise ValueError("edf_pwr_percent must be in range 0..100")

    payload = struct.pack(
        "<bbbbb",
        pwr,
        encode_servo_angle(x_plus_angle_deg),
        encode_servo_angle(x_minus_angle_deg),
        encode_servo_angle(y_plus_angle_deg),
        encode_servo_angle(y_minus_angle_deg),
    )

    return build_frame(
        version=PROTOCOL_VER,
        flags=FLAG_DATA,
        src=ID_PC,
        dst=ID_DRONE,
        opcode=OPT_CAL_PARAM,
        payload=payload,
    )


def send_power(
    ser: serial.Serial,
    power: int,
    *,
    x_plus_angle_deg: int = 0,
    x_minus_angle_deg: int = 0,
    y_plus_angle_deg: int = 0,
    y_minus_angle_deg: int = 0,
) -> None:
    frame = build_calib_frame(
        power,
        x_plus_angle_deg=x_plus_angle_deg,
        x_minus_angle_deg=x_minus_angle_deg,
        y_plus_angle_deg=y_plus_angle_deg,
        y_minus_angle_deg=y_minus_angle_deg,
    )
    ser.write(frame)
    ser.flush()
    print(f"TX power={power:3d}% | {frame.hex(' ')}")


def main() -> int:
    # Example usage:
    #   python.exe edf_set_power.py --port COM5 --baud 9600 --power 30
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Drone UART COM port (e.g. COM5)")
    ap.add_argument("--baud", type=int, default=9600, help="Drone UART baud rate (default: 9600)")
    ap.add_argument("--power", type=int, default=None, help="EDF power percent 0..100 (omit for interactive)")
    ap.add_argument("--x-plus", type=int, default=0, help="X+ servo angle in degrees (default: 0)")
    ap.add_argument("--x-minus", type=int, default=0, help="X- servo angle in degrees (default: 0)")
    ap.add_argument("--y-plus", type=int, default=0, help="Y+ servo angle in degrees (default: 0)")
    ap.add_argument("--y-minus", type=int, default=0, help="Y- servo angle in degrees (default: 0)")
    ap.add_argument("--rx-ms", type=int, default=0, help="After TX, read & print raw bytes for N ms (default: 0)")
    args = ap.parse_args()

    with serial.Serial(
        args.port,
        baudrate=args.baud,
        timeout=0.1,
        write_timeout=0.5,
        dsrdtr=False,
        rtscts=False,
    ) as ser:
        time.sleep(0.05)
        ser.reset_input_buffer()

        def tx(p: int) -> None:
            send_power(
                ser,
                p,
                x_plus_angle_deg=args.x_plus,
                x_minus_angle_deg=args.x_minus,
                y_plus_angle_deg=args.y_plus,
                y_minus_angle_deg=args.y_minus,
            )
            if args.rx_ms > 0:
                deadline = time.monotonic() + (args.rx_ms / 1000.0)
                buf = bytearray()
                while time.monotonic() < deadline:
                    chunk = ser.read(ser.in_waiting or 1)
                    if chunk:
                        buf.extend(chunk)
                if buf:
                    print(f"RX ({len(buf)} bytes): {bytes(buf).hex(' ')}")

        if args.power is not None:
            tx(args.power)
            return 0

        print("Interactive mode: enter EDF power percent 0..100, or 'q' to quit.")
        while True:
            s = input("power%> ").strip().lower()
            if s in {"q", "quit", "exit"}:
                break
            if not s:
                continue
            try:
                pwr = int(s)
            except ValueError:
                print("Enter an integer 0..100 (or q).")
                continue
            try:
                tx(pwr)
            except Exception as e:  # noqa: BLE001
                print(f"ERR: {e}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
