import argparse
import sys
import time

import serial
from serial.serialutil import SerialException
from serial.tools import list_ports


def list_available_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("Available ports: <none>")
        return
    print("Available ports:")
    for p in ports:
        desc = (p.description or "").strip()
        hwid = (p.hwid or "").strip()
        print(f"  - {p.device:>6s}  {desc}  ({hwid})")


def _set_dtr_rts(ser: serial.Serial, *, dtr: bool | None = None, rts: bool | None = None) -> None:
    try:
        if dtr is not None:
            ser.dtr = bool(dtr)
        if rts is not None:
            ser.rts = bool(rts)
    except Exception:
        # Some backends/boards may not support these toggles.
        pass


def read_any_data(ser: serial.Serial, *, listen_s: float) -> bytes:
    deadline = time.time() + listen_s
    out = bytearray()
    while time.time() < deadline:
        try:
            waiting = int(getattr(ser, "in_waiting", 0) or 0)
        except Exception:
            waiting = 0

        if waiting > 0:
            chunk = ser.read(waiting)
            if chunk:
                out.extend(chunk)
                continue

        line = ser.readline()
        if line:
            out.extend(line)
            continue

        time.sleep(0.01)

    return bytes(out)


def wait_for_banner(
    ser: serial.Serial,
    *,
    listen_s: float,
    banner_substrings: tuple[bytes, ...] = (b"Arduino", b"HX711", b"Format:", b"#"),
) -> bytes:
    """
    Read until any recognizable banner bytes appear (or until timeout).
    Returns whatever was captured (may be empty).
    """
    deadline = time.time() + listen_s
    buf = bytearray()
    while time.time() < deadline:
        chunk = read_any_data(ser, listen_s=0.25)
        if chunk:
            buf.extend(chunk)
            lowered = bytes(buf).lower()
            if any(s.lower() in lowered for s in banner_substrings):
                break
        else:
            time.sleep(0.05)
    return bytes(buf)


def main() -> int:
    parser = argparse.ArgumentParser(description="Arduino serial sanity-check (terminal).")
    parser.add_argument("--port", default="COM11", help="Arduino COM port (default: COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--cmd", default="T", help="Command to send (default: T)")
    parser.add_argument(
        "--boot-wait",
        type=float,
        default=8.0,
        help="Seconds to wait for any startup/banner output after opening (default: 8)",
    )
    parser.add_argument(
        "--pre-cmd-sleep",
        type=float,
        default=0.0,
        help="Extra sleep seconds before sending the command (default: 0)",
    )
    parser.add_argument("--listen", type=float, default=20.0, help="Listen timeout seconds (default: 20)")
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Do not toggle DTR/RTS (some boards reset on open).",
    )
    parser.add_argument(
        "--reopen-after-reset",
        action="store_true",
        help="Close and reopen the port after toggling DTR (helps boards that briefly disconnect).",
    )
    args = parser.parse_args()

    list_available_ports()
    print(f"\nOpening {args.port} @ {args.baud} ...")

    def open_serial() -> serial.Serial:
        return serial.Serial(
            args.port,
            args.baud,
            timeout=0.2,
            write_timeout=0.5,
            dsrdtr=False,
            rtscts=False,
        )

    try:
        ser = open_serial()
    except SerialException as e:
        print(f"ERROR: failed to open {args.port}: {e}")
        return 2

    try:
        if not args.no_reset:
            # Match Arduino Serial Monitor behavior and handle sketches with `while (!Serial) {}`
            _set_dtr_rts(ser, dtr=False, rts=False)
            time.sleep(0.1)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            _set_dtr_rts(ser, dtr=True)

            # Many boards reset on DTR; allow time for boot + banner prints.
            time.sleep(0.5)

            if args.reopen_after_reset:
                print("Reopening port after reset ...")
                try:
                    ser.close()
                except Exception:
                    pass

                reopen_deadline = time.time() + 10.0
                last_err: str | None = None
                while time.time() < reopen_deadline:
                    try:
                        ser = open_serial()
                        break
                    except SerialException as e:
                        last_err = str(e)
                        time.sleep(0.25)
                else:
                    print(f"ERROR: failed to reopen {args.port} after reset: {last_err}")
                    list_available_ports()
                    return 2

        # Read any startup/banner output (helps confirm the sketch actually runs).
        print(f"Waiting for startup output (up to {args.boot_wait:.1f}s) ...")
        boot_data = wait_for_banner(ser, listen_s=float(args.boot_wait))
        if boot_data:
            print(f"Startup bytes: {len(boot_data)}")
            boot_text = boot_data.decode("utf-8", errors="ignore").replace("\r\n", "\n").replace("\r", "\n")
            if boot_text.strip():
                print("STARTUP TEXT:")
                print(boot_text)
        else:
            print("No startup output received.")

        if float(args.pre_cmd_sleep) > 0:
            time.sleep(float(args.pre_cmd_sleep))

        cmd_bytes = args.cmd.encode("utf-8", errors="ignore")
        print(f"Sending {cmd_bytes!r} ...")
        ser.write(cmd_bytes)
        ser.flush()

        print("Listening ...")
        data = read_any_data(ser, listen_s=float(args.listen))

        if not data:
            print(
                f"ERROR: no data received within {args.listen:.1f}s.\n"
                "- Close Arduino Serial Monitor/other serial tools.\n"
                "- Verify COM port (it can change after unplug/replug).\n"
                "- Power-cycle the board.\n"
                "- If your sketch has `while (!Serial) {}`, keep DTR enabled (default behavior here).\n"
                "- If this board is a native-USB Arduino that re-enumerates on reset, try `--no-reset`."
            )
            return 1

        print(f"Received {len(data)} bytes.")
        print("HEX:", data.hex(" "))
        text = data.decode("utf-8", errors="ignore").replace("\r\n", "\n").replace("\r", "\n")
        if text.strip():
            print("\nTEXT:")
            print(text)

        if b"TARE_DONE" in data:
            print("\nOK: saw TARE_DONE")
            return 0

        print("\nOK: received data (no TARE_DONE found).")
        return 0

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
