#!/usr/bin/env python3
"""
PX100: reset + run a CC discharge test + save readings to CSV.

Changes vs earlier:
- Reset uses value 0.0 (per your device/driver)
- Reads current settings first and only sends SET commands if needed (tolerances)
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict

import serial

# Run from repo root so these imports resolve:
from instruments.instrument import Instrument
from instruments.px100 import PX100


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def safe_float(x):
    try:
        return float(x)
    except Exception:
        return None


class SerialRawAdapter:
    """
    Adapter for PX100 driver transport expectations.
    """
    def __init__(self, port: str, baud: int = 9600, timeout: float = 2.0):
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def __repr__(self) -> str:
        return f"SerialRawAdapter(port={self._ser.port!r}, baud={self._ser.baudrate}, timeout={self._ser.timeout})"

    def write_raw(self, b: bytes) -> int:
        return self._ser.write(b)

    def read_raw(self, n: int) -> bytes:
        return self._ser.read(n)

    def write_bytes(self, b: bytes) -> int:
        return self.write_raw(b)

    def read_bytes(self, n: int) -> bytes:
        return self.read_raw(n)

    @property
    def bytes_in_buffer(self) -> int:
        try:
            return int(self._ser.in_waiting)
        except Exception:
            return 0

    def flush(self) -> None:
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


def try_command(dev: PX100, cmd_name: str, value):
    """
    Call the driver's command method in a couple common forms.
    """
    try:
        return dev.command(cmd_name, value)
    except TypeError:
        pass

    try:
        return dev.command({cmd_name: value})
    except TypeError:
        pass

    raise


def normalize_readall(data) -> Dict[str, Any]:
    """
    Normalize readAll() output into a dict so we can work with it.
    """
    if data is None:
        return {}
    if isinstance(data, dict):
        return data

    out = {}
    for k in dir(data):
        if k.startswith("_"):
            continue
        v = getattr(data, k, None)
        if callable(v):
            continue
        out[k] = v
    return out


def nearly_equal(a: float | None, b: float | None, tol: float) -> bool:
    if a is None or b is None:
        return False
    return abs(a - b) <= tol


def set_if_needed(dev: PX100, cmd_name: str, current_val, desired_val, tol: float, label: str):
    cur = safe_float(current_val)
    des = safe_float(desired_val)

    # If we can't read current, just set it (safe default).
    if cur is None or des is None:
        print(f"[{utc_now_iso()}] Setting {label} (could not compare).")
        try_command(dev, cmd_name, desired_val)
        return True

    if nearly_equal(cur, des, tol):
        print(f"[{utc_now_iso()}] {label} already {cur} (target {des}); skipping set.")
        return False

    print(f"[{utc_now_iso()}] Setting {label}: {cur} -> {des}")
    try_command(dev, cmd_name, desired_val)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM5)")
    ap.add_argument("--current-a", type=float, required=True, help="Discharge current in amps (CC mode)")
    ap.add_argument("--cutoff-v", type=float, required=True, help="Cutoff voltage in volts")
    ap.add_argument("--timer-s", type=int, default=0, help="Optional max duration in seconds (0 = no timer)")
    ap.add_argument("--interval-s", type=float, default=1.0, help="Sampling interval seconds")
    ap.add_argument("--out", required=True, help="Output CSV path")
    ap.add_argument("--no-reset", action="store_true", help="Skip reset step")
    ap.add_argument("--timeout-s", type=float, default=2.0, help="Serial read timeout seconds (default: 2.0)")
    ap.add_argument("--tol-v", type=float, default=0.01, help="Voltage compare tolerance in V (default: 0.01)")
    ap.add_argument("--tol-a", type=float, default=0.01, help="Current compare tolerance in A (default: 0.01)")
    args = ap.parse_args()

    transport = SerialRawAdapter(args.port, baud=9600, timeout=args.timeout_s)
    dev = PX100(transport)

    # Allow USB-serial to settle, then clear buffers
    time.sleep(0.5)
    transport.flush()

    # Reset (your device expects 0.0)
    if not args.no_reset:
        try:
            try_command(dev, Instrument.COMMAND_RESET, 0.0)
        except Exception as e:
            print(f"[{utc_now_iso()}] Warning: reset failed: {e}", file=sys.stderr)
        time.sleep(0.5)

    # Read current state/settings
    state = normalize_readall(dev.readAll())

    # Only set if needed
    set_if_needed(
        dev,
        Instrument.COMMAND_SET_CURRENT,
        current_val=state.get("set_current"),
        desired_val=float(args.current_a),
        tol=args.tol_a,
        label="set_current (A)",
    )
    set_if_needed(
        dev,
        Instrument.COMMAND_SET_VOLTAGE,
        current_val=state.get("set_voltage"),
        desired_val=float(args.cutoff_v),
        tol=args.tol_v,
        label="set_voltage/cutoff (V)",
    )

    # Timer: if you use it, set it; otherwise skip (and we don't try to compare datetime.time objects)
    if args.timer_s > 0:
        print(f"[{utc_now_iso()}] Setting timer to {args.timer_s}s")
        try_command(dev, Instrument.COMMAND_SET_TIMER, int(args.timer_s))

    # Enable load (start)
    try_command(dev, Instrument.COMMAND_ENABLE, True)
    time.sleep(0.2)

    start = time.time()
    wrote_header = False

    with open(args.out, "w", newline="") as f:
        writer = None

        print(f"[{utc_now_iso()}] Running… logging to {args.out}")
        try:
            while True:
                t = time.time() - start

                raw = dev.readAll()
                row = normalize_readall(raw)

                if not wrote_header:
                    preferred = [
                        "is_on", "voltage", "current", "temp",
                        "cap_ah", "cap_wh", "time",
                        "set_current", "set_voltage", "set_timer",
                    ]
                    extras = [k for k in row.keys() if k not in preferred]
                    fieldnames = ["t_s", "timestamp_utc"] + [k for k in preferred if k in row] + sorted(extras)
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    wrote_header = True

                csv_row = {"t_s": round(t, 3), "timestamp_utc": utc_now_iso()}
                csv_row.update(row)
                writer.writerow(csv_row)
                f.flush()

                # Stop by timer
                if args.timer_s > 0 and t >= args.timer_s:
                    print(f"[{utc_now_iso()}] Timer reached ({args.timer_s}s). Stopping.")
                    break

                # Stop by cutoff voltage
                v = safe_float(row.get("voltage"))
                if v is not None and v <= args.cutoff_v:
                    print(f"[{utc_now_iso()}] Cutoff reached (V={v} <= {args.cutoff_v}). Stopping.")
                    break

                time.sleep(max(0.05, args.interval_s))

        except KeyboardInterrupt:
            print(f"\n[{utc_now_iso()}] Ctrl+C received — stopping test.")


        finally:
            # Always disable load
            try:
                try_command(dev, Instrument.COMMAND_ENABLE, False)
            except Exception as e:
                print(f"[{utc_now_iso()}] Warning: disable failed: {e}", file=sys.stderr)

            try:
                transport.close()
            except Exception:
                pass

    print(f"[{utc_now_iso()}] Done. Saved: {args.out}")


if __name__ == "__main__":
    main()

