#!/usr/bin/env python3
import glob
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# ---- NEW PX100 driver imports (your working code) ----
# These must be available on PYTHONPATH (run from your repo root or install the package)
from instruments.instrument import Instrument
from instruments.px100 import PX100 as PX100_Driver


# -------------------- Port helpers --------------------
def list_serial_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for dev in glob.glob(pat):
            if dev not in ports:
                ports.append(dev)
    return sorted(ports)


def parse_tray_status_line(line: str) -> dict:
    out = {}
    line = line.strip()
    if not line.startswith("STATUS"):
        return out
    parts = line.split()
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            out[k.strip()] = v.strip()
    return out


# -------------------- Tray (Arduino) worker --------------------
class TraySerialWorker(threading.Thread):
    def __init__(self, rx_queue: queue.Queue, tx_queue: queue.Queue):
        super().__init__(daemon=True)
        self.rx_queue = rx_queue
        self.tx_queue = tx_queue
        self.ser = None
        self._stop_evt = threading.Event()   # don't shadow Thread._stop
        self._connected = threading.Event()
        self.status_poll_interval = 0.5
        self._last_poll = 0.0

    def connect(self, port: str, baud: int = 9600):
        self.tx_queue.put(("__CONNECT__", {"port": port, "baud": baud}))

    def disconnect(self):
        self.tx_queue.put(("__DISCONNECT__", {}))

    def is_connected(self) -> bool:
        return self._connected.is_set()

    def send(self, cmd: str):
        self.tx_queue.put(("CMD", {"cmd": cmd}))

    def stop(self):
        self._stop_evt.set()
        self.disconnect()

    def _do_connect(self, port: str, baud: int):
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            time.sleep(1.2)  # allow Arduino auto-reset
            self._connected.set()
            self.rx_queue.put(("INFO", f"Tray connected: {port} @ {baud}"))
        except Exception as e:
            self._connected.clear()
            self.ser = None
            self.rx_queue.put(("ERR", f"Tray connect failed: {e}"))

    def _do_disconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._connected.clear()
        self.rx_queue.put(("INFO", "Tray disconnected"))

    def _write_line(self, s: str):
        if not self.ser or not self.is_connected():
            return
        try:
            self.ser.write((s.strip() + "\n").encode("utf-8"))
        except Exception as e:
            self.rx_queue.put(("ERR", f"Tray write failed: {e}"))
            self._do_disconnect()

    def _read_lines(self):
        if not self.ser or not self.is_connected():
            return
        try:
            while True:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.rx_queue.put(("LINE", line))
        except Exception as e:
            self.rx_queue.put(("ERR", f"Tray read failed: {e}"))
            self._do_disconnect()

    def run(self):
        while not self._stop_evt.is_set():
            try:
                msg_type, payload = self.tx_queue.get_nowait()
            except queue.Empty:
                msg_type, payload = None, None

            if msg_type == "__CONNECT__":
                self._do_connect(payload["port"], payload["baud"])
            elif msg_type == "__DISCONNECT__":
                self._do_disconnect()
            elif msg_type == "CMD":
                cmd = payload["cmd"]
                if self.is_connected():
                    self._write_line(cmd)
                    self.rx_queue.put(("SENT", cmd))
                else:
                    self.rx_queue.put(("ERR", "Tray not connected"))

            now = time.time()
            if self.is_connected() and (now - self._last_poll) >= self.status_poll_interval:
                self._last_poll = now
                self._write_line("STATUS")

            self._read_lines()
            time.sleep(0.01)


# -------------------- DPS5020 (Modbus) --------------------
class DPS5020:
    REG_IOUT = 0x0003
    REG_OUT_EN = 0x0009

    def __init__(self, port: str, baudrate: int = 9600, slave_id: int = 1):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.client = None
        self.connected = False
        self.lock = threading.Lock()

    def connect(self) -> bool:
        with self.lock:
            try:
                self.client = ModbusSerialClient(
                    method="rtu",
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.5,
                    parity="N",
                    stopbits=1,
                    bytesize=8,
                )
                self.connected = bool(self.client.connect())
                return self.connected
            except Exception:
                self.connected = False
                self.client = None
                return False

    def disconnect(self):
        with self.lock:
            try:
                if self.client:
                    self.client.close()
            except Exception:
                pass
            self.client = None
            self.connected = False

    def _read1(self, reg: int):
        if not self.connected or not self.client:
            return None
        try:
            rr = self.client.read_holding_registers(address=reg, count=1, slave=self.slave_id)
            if rr and not rr.isError():
                return rr.registers[0]
        except Exception:
            return None
        return None

    def _write1(self, reg: int, value: int) -> bool:
        if not self.connected or not self.client:
            return False
        try:
            wr = self.client.write_register(address=reg, value=value, slave=self.slave_id)
            return bool(wr) and (not wr.isError())
        except Exception:
            return False

    def read_current_a(self):
        v = self._read1(self.REG_IOUT)
        return None if v is None else (v / 100.0)

    def read_output_enabled(self):
        v = self._read1(self.REG_OUT_EN)
        return None if v is None else (1 if v != 0 else 0)

    def set_output_enabled(self, enabled: bool) -> bool:
        return self._write1(self.REG_OUT_EN, 1 if enabled else 0)

    def toggle_output(self):
        cur = self.read_output_enabled()
        if cur is None:
            return False, None
        ok = self.set_output_enabled(cur == 0)
        return ok, (0 if cur else 1)


# -------------------- PX100 (your working driver) --------------------
def safe_float(x):
    try:
        return float(x)
    except Exception:
        return None


def nearly_equal(a, b, tol: float) -> bool:
    aa = safe_float(a)
    bb = safe_float(b)
    if aa is None or bb is None:
        return False
    return abs(aa - bb) <= tol


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


def normalize_readall(data) -> dict:
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


def try_command(dev: PX100_Driver, cmd_name: str, value):
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


class PX100Controller:
    """
    Thin wrapper used by the GUI.
    """
    def __init__(self, port: str, baud: int = 9600, timeout_s: float = 2.0):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.transport = None
        self.dev = None
        self.connected = False
        self.lock = threading.Lock()

        # tolerances for "set if needed"
        self.tol_v = 0.01
        self.tol_a = 0.01

    def connect(self) -> bool:
        with self.lock:
            try:
                self.transport = SerialRawAdapter(self.port, baud=self.baud, timeout=self.timeout_s)
                self.dev = PX100_Driver(self.transport)
                time.sleep(0.5)
                self.transport.flush()
                self.connected = True
                return True
            except Exception:
                self.transport = None
                self.dev = None
                self.connected = False
                return False

    def disconnect(self):
        with self.lock:
            try:
                if self.dev:
                    try:
                        try_command(self.dev, Instrument.COMMAND_ENABLE, False)
                    except Exception:
                        pass
            except Exception:
                pass
            try:
                if self.transport:
                    self.transport.close()
            except Exception:
                pass
            self.transport = None
            self.dev = None
            self.connected = False

    def read_all(self) -> dict:
        with self.lock:
            if not self.connected or not self.dev:
                return {}
            try:
                return normalize_readall(self.dev.readAll())
            except Exception:
                return {}

    def probe(self) -> bool:
        st = self.read_all()
        return bool(st)

    def reset(self) -> bool:
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_RESET, 0.0)  # your device expects 0.0
                time.sleep(0.5)
                return True
            except Exception:
                return False

    def set_if_needed(self, cmd_name: str, current_val, desired_val, tol: float) -> bool:
        with self.lock:
            if not self.connected or not self.dev:
                return False

            # If can't compare, just set
            cur = safe_float(current_val)
            des = safe_float(desired_val)
            if cur is None or des is None:
                try:
                    try_command(self.dev, cmd_name, desired_val)
                    return True
                except Exception:
                    return False

            if nearly_equal(cur, des, tol):
                return True  # already fine

            try:
                try_command(self.dev, cmd_name, desired_val)
                return True
            except Exception:
                return False

    def set_current_a(self, current_a: float) -> bool:
        st = self.read_all()
        return self.set_if_needed(
            Instrument.COMMAND_SET_CURRENT,
            current_val=st.get("set_current"),
            desired_val=float(current_a),
            tol=self.tol_a,
        )

    def set_cutoff_v(self, cutoff_v: float) -> bool:
        st = self.read_all()
        # driver calls it set_voltage, but we treat as cutoff target
        return self.set_if_needed(
            Instrument.COMMAND_SET_VOLTAGE,
            current_val=st.get("set_voltage"),
            desired_val=float(cutoff_v),
            tol=self.tol_v,
        )

    def set_timer_s(self, timer_s: int) -> bool:
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_SET_TIMER, int(timer_s))
                return True
            except Exception:
                return False

    def set_load_enabled(self, enabled: bool) -> bool:
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_ENABLE, bool(enabled))
                return True
            except Exception:
                return False

    def start_capacity_test(self, current_a: float, cutoff_v: float, timer_s: int = 0) -> bool:
        # safe sequence: disable -> set -> (timer) -> enable
        ok = self.set_load_enabled(False)
        time.sleep(0.15)
        ok = ok and self.set_current_a(current_a)
        time.sleep(0.10)
        ok = ok and self.set_cutoff_v(cutoff_v)
        time.sleep(0.10)
        if timer_s and timer_s > 0:
            ok = ok and self.set_timer_s(timer_s)
            time.sleep(0.10)
        ok = ok and self.set_load_enabled(True)
        return ok


# -------------------- GUI --------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Cell Tray + DPS5020 + PX100")
        self.geometry("1120x700")

        # Tray
        self.tray_rx = queue.Queue()
        self.tray_tx = queue.Queue()
        self.tray = TraySerialWorker(self.tray_rx, self.tray_tx)
        self.tray.start()

        # PSU + Load
        self.dps = None
        self.px100 = None
        self._px_poll_step = 0

        # UI vars
        self.tray_port = tk.StringVar(value="")
        self.tray_baud = tk.StringVar(value="9600")
        self.tray_conn_var = tk.StringVar(value="Tray: Disconnected")

        self.psu_port = tk.StringVar(value="")
        self.psu_baud = tk.StringVar(value="9600")
        self.psu_slave = tk.StringVar(value="1")
        self.psu_conn_var = tk.StringVar(value="PSU: Disconnected")
        self.psu_current_var = tk.StringVar(value="--.- A")
        self.psu_out_var = tk.StringVar(value="--")

        self.px_port = tk.StringVar(value="")
        self.px_baud = tk.StringVar(value="9600")
        self.px_conn_var = tk.StringVar(value="PX100: Disconnected")

        self.px_set_current = tk.StringVar(value="1.00")
        self.px_set_cutoff = tk.StringVar(value="3.00")

        self.px_v_var = tk.StringVar(value="--.- V")
        self.px_i_var = tk.StringVar(value="--.- A")
        self.px_mah_var = tk.StringVar(value="---- mAh")
        self.px_mwh_var = tk.StringVar(value="---- mWh")
        self.px_time_var = tk.StringVar(value="--:--:--")
        self.px_temp_var = tk.StringVar(value="-- °C")

        self.tray_status_vars = {
            "homeClosed": tk.StringVar(value="-"),
            "aligned": tk.StringVar(value="-"),
            "detectClosed": tk.StringVar(value="-"),
            "contacts": tk.StringVar(value="-"),
            "r1": tk.StringVar(value="-"),
            "r2": tk.StringVar(value="-"),
        }

        # ---------------- Auto-next settings/state ----------------
        self.auto_next_enabled = tk.BooleanVar(value=False)
        self.auto_next_threshold_a = 0.10  # 100 mA

        self.auto_next_stabilize_s = 10.0   # require low current for 10s
        self._low_current_since = None      # timestamp when current first went <= threshold

        self._auto_next_in_progress = False
        self._auto_next_last_step_ts = 0.0

        self._tray_status_latest = {}
        self._tray_status_ts = 0.0
        # ----------------------------------------------------------

        self._build_ui()
        self._refresh_ports()

        self.after(50, self._pump_tray_rx)
        self.after(400, self._poll_psu)
        self.after(400, self._poll_px100)

    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("TButton", font=("Sans", 18))
        style.configure("TLabel", font=("Sans", 14))
        style.configure("Conn.TLabel", font=("Sans", 11))
        style.configure("Conn.TButton", font=("Sans", 11))

        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        # --- Main layout ---
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True, padx=(0, 10))

        right = ttk.Frame(main)
        right.pack(side="left", fill="both", expand=True)

        # --- Tray controls ---
        tray_ctrl = ttk.LabelFrame(left, text="Tray Controls", padding=10)
        tray_ctrl.pack(fill="x")

        ttk.Button(tray_ctrl, text="LOAD", command=self._cmd_load).pack(fill="x", pady=5)
        ttk.Button(tray_ctrl, text="NEXT CELL", command=lambda: self._tray_send("NEXT")).pack(fill="x", pady=5)
        ttk.Button(tray_ctrl, text="UNLOAD", command=lambda: self._tray_send("UNLOAD")).pack(fill="x", pady=5)
        ttk.Separator(tray_ctrl).pack(fill="x", pady=10)
        ttk.Button(tray_ctrl, text="STOP", command=lambda: self._tray_send("STOP")).pack(fill="x", pady=5)

        # --- PSU controls ---
        psu_ctrl = ttk.LabelFrame(left, text="PSU Controls", padding=10)
        psu_ctrl.pack(fill="x", pady=(10, 0))

        psu_btns = ttk.Frame(psu_ctrl)
        psu_btns.pack(fill="x")
        ttk.Button(psu_btns, text="PSU ON", command=self._psu_on).pack(
            side="left", expand=True, fill="x", padx=(0, 6)
        )
        ttk.Button(psu_btns, text="PSU OFF", command=self._psu_off).pack(
            side="left", expand=True, fill="x", padx=(0, 6)
        )
        ttk.Button(psu_btns, text="PSU TOGGLE", command=self._psu_toggle).pack(
            side="left", expand=True, fill="x"
        )

        ttk.Checkbutton(
            psu_ctrl,
            text="Auto load next cell while PSU current ≤ 100 mA (after 10s stable)",
            variable=self.auto_next_enabled,
        ).pack(anchor="w", pady=(8, 0))

        # --- PX100 controls ---
        px_ctrl = ttk.LabelFrame(left, text="PX100 Capacity Test", padding=10)
        px_ctrl.pack(fill="x", pady=(10, 0))

        row = ttk.Frame(px_ctrl)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text="Discharge current (A):").pack(side="left")
        ttk.Entry(row, textvariable=self.px_set_current, width=8).pack(side="right")

        row2 = ttk.Frame(px_ctrl)
        row2.pack(fill="x", pady=4)
        ttk.Label(row2, text="Cutoff voltage (V):").pack(side="left")
        ttk.Entry(row2, textvariable=self.px_set_cutoff, width=8).pack(side="right")

        px_btns = ttk.Frame(px_ctrl)
        px_btns.pack(fill="x", pady=(8, 0))
        ttk.Button(px_btns, text="START TEST", command=self._px_start).pack(
            side="left", expand=True, fill="x", padx=(0, 6)
        )
        ttk.Button(px_btns, text="RESET", command=self._px_reset).pack(
            side="left", expand=True, fill="x", padx=(0, 6)
        )
        ttk.Button(px_btns, text="STOP LOAD", command=self._px_stop).pack(side="left", expand=True, fill="x")

        # --- Status panel ---
        status = ttk.LabelFrame(right, text="Status (auto update)", padding=10)
        status.pack(fill="x")

        grid = ttk.Frame(status)
        grid.pack(fill="x")

        def srow(r, label, var):
            ttk.Label(grid, text=label).grid(row=r, column=0, sticky="w", padx=(0, 10), pady=5)
            ttk.Label(grid, textvariable=var, font=("Sans", 16, "bold")).grid(
                row=r, column=1, sticky="w", pady=5
            )

        # Tray status
        srow(0, "Tray detectClosed:", self.tray_status_vars["detectClosed"])
        srow(1, "Tray homeClosed:", self.tray_status_vars["homeClosed"])
        srow(2, "Tray aligned:", self.tray_status_vars["aligned"])
        srow(3, "Tray contacts:", self.tray_status_vars["contacts"])
        srow(4, "Tray relay1:", self.tray_status_vars["r1"])
        srow(5, "Tray relay2:", self.tray_status_vars["r2"])

        ttk.Separator(status).pack(fill="x", pady=10)

        # PSU status
        srow(6, "PSU output:", self.psu_out_var)
        srow(7, "PSU current:", self.psu_current_var)

        ttk.Separator(status).pack(fill="x", pady=10)

        # PX100 status
        srow(8, "PX100 voltage:", self.px_v_var)
        srow(9, "PX100 current:", self.px_i_var)
        srow(10, "PX100 capacity:", self.px_mah_var)
        srow(11, "PX100 energy:", self.px_mwh_var)
        srow(12, "PX100 time:", self.px_time_var)
        srow(13, "PX100 temp:", self.px_temp_var)

        # --- Log ---
        logf = ttk.LabelFrame(right, text="Log", padding=10)
        logf.pack(fill="both", expand=True, pady=(10, 0))

        self.log = tk.Text(logf, height=10, wrap="word", font=("Sans", 12))
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")

        # ---------- Bottom connection bar ----------
        conn_bar = ttk.LabelFrame(self, text="Connections", padding=6)
        conn_bar.pack(side="bottom", fill="x")

        def clabel(parent, text):
            return ttk.Label(parent, text=text, style="Conn.TLabel")

        def cbutton(parent, text, cmd):
            return ttk.Button(parent, text=text, command=cmd, style="Conn.TButton")

        # ---- Tray ----
        tray_row = ttk.Frame(conn_bar)
        tray_row.pack(fill="x", pady=2)

        clabel(tray_row, "Tray").pack(side="left", padx=(0, 6))
        self.tray_combo = ttk.Combobox(tray_row, textvariable=self.tray_port, width=12, state="readonly")
        self.tray_combo.pack(side="left")
        ttk.Entry(tray_row, textvariable=self.tray_baud, width=6).pack(side="left", padx=4)
        cbutton(tray_row, "Connect", self._tray_connect).pack(side="left", padx=2)
        cbutton(tray_row, "Disc", self._tray_disconnect).pack(side="left", padx=2)
        ttk.Label(tray_row, textvariable=self.tray_conn_var, style="Conn.TLabel").pack(side="right")

        # ---- PSU ----
        psu_row = ttk.Frame(conn_bar)
        psu_row.pack(fill="x", pady=2)

        clabel(psu_row, "PSU").pack(side="left", padx=(0, 6))
        self.psu_combo = ttk.Combobox(psu_row, textvariable=self.psu_port, width=12, state="readonly")
        self.psu_combo.pack(side="left")
        ttk.Entry(psu_row, textvariable=self.psu_baud, width=6).pack(side="left", padx=4)
        ttk.Entry(psu_row, textvariable=self.psu_slave, width=4).pack(side="left", padx=4)
        cbutton(psu_row, "Connect", self._psu_connect).pack(side="left", padx=2)
        cbutton(psu_row, "Disc", self._psu_disconnect).pack(side="left", padx=2)
        ttk.Label(psu_row, textvariable=self.psu_conn_var, style="Conn.TLabel").pack(side="right")

        # ---- PX100 ----
        px_row = ttk.Frame(conn_bar)
        px_row.pack(fill="x", pady=2)

        clabel(px_row, "PX100").pack(side="left", padx=(0, 6))
        self.px_combo = ttk.Combobox(px_row, textvariable=self.px_port, width=12, state="readonly")
        self.px_combo.pack(side="left")
        ttk.Entry(px_row, textvariable=self.px_baud, width=6).pack(side="left", padx=4)
        cbutton(px_row, "Connect", self._px_connect).pack(side="left", padx=2)
        cbutton(px_row, "Disc", self._px_disconnect).pack(side="left", padx=2)
        ttk.Label(px_row, textvariable=self.px_conn_var, style="Conn.TLabel").pack(side="right")

    def _log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.tray_combo["values"] = ports
        self.psu_combo["values"] = ports
        self.px_combo["values"] = ports

        if ports and self.tray_port.get() not in ports:
            self.tray_port.set(ports[0])
        if ports and self.psu_port.get() not in ports:
            self.psu_port.set(ports[0])
        if ports and self.px_port.get() not in ports:
            self.px_port.set(ports[0])

    # ---------------- Auto-next helpers ----------------
    def _tray_status_fresh(self, max_age_s: float = 2.0) -> bool:
        return (time.time() - self._tray_status_ts) <= max_age_s

    def _tray_cell_looks_loaded(self) -> bool:
        """
        Best-effort: treat "loaded" if we have recent status and
        any of contacts/aligned/detectClosed == 1.
        Adjust this if your firmware uses different semantics.
        """
        if not self._tray_status_fresh():
            return False
        st = self._tray_status_latest or {}

        def as_int(x):
            try:
                return int(str(x).strip())
            except Exception:
                return None

        contacts = as_int(st.get("contacts"))
        aligned = as_int(st.get("aligned"))
        detect = as_int(st.get("detectClosed"))

        return (contacts == 1) or (aligned == 1) or (detect == 1)

    def _auto_wait_for_cell(self, confirm_timeout_ms=4500, check_every_ms=250):
        start_ts = time.time()

        def check():
            if not self.auto_next_enabled.get():
                self._auto_next_in_progress = False
                return

            if self._tray_cell_looks_loaded():
                self._log("AUTO: Next cell loaded")
                self._auto_next_in_progress = False
                return

            if (time.time() - start_ts) * 1000.0 >= confirm_timeout_ms:
                self._log("AUTO: Could not confirm a new cell (tray empty or jam) -> auto mode stopped")
                self.auto_next_enabled.set(False)
                self._auto_next_in_progress = False
                return

            self.after(check_every_ms, check)

        self.after(check_every_ms, check)

    def _auto_advance_step(self):
        """
        While auto is enabled and current has been low for 10s, advance to the next cell.
        Stops automatically if tray can't confirm a new cell.
        """
        if not self.auto_next_enabled.get():
            self._auto_next_in_progress = False
            return

        if not self.tray.is_connected():
            self._log("AUTO: Tray not connected -> auto mode stopped")
            self.auto_next_enabled.set(False)
            self._auto_next_in_progress = False
            return

        if self._auto_next_in_progress:
            return

        self._auto_next_in_progress = True
        self._auto_next_last_step_ts = time.time()

        self._log("AUTO: Current low & stable -> loading next cell")
        self._tray_send("NEXT")
        self.after(250, lambda: self._tray_send("RECONNECT"))

        self._auto_wait_for_cell(confirm_timeout_ms=4500, check_every_ms=250)

    # -------- Tray actions --------
    def _tray_connect(self):
        port = self.tray_port.get().strip()
        if not port:
            messagebox.showwarning("Tray", "Select a tray serial port.")
            return
        try:
            baud = int(self.tray_baud.get().strip())
        except ValueError:
            messagebox.showwarning("Tray", "Tray baud must be a number.")
            return
        self.tray.connect(port, baud)

    def _tray_disconnect(self):
        self.tray.disconnect()

    def _tray_send(self, cmd: str):
        self.tray.send(cmd)

    def _cmd_load(self):
        # LOAD = HOME then RECONNECT
        self._tray_send("HOME")
        self.after(250, lambda: self._tray_send("RECONNECT"))

    def _pump_tray_rx(self):
        try:
            while True:
                mtype, payload = self.tray_rx.get_nowait()
                if mtype == "INFO":
                    self.tray_conn_var.set(payload)
                    self._log(payload)
                elif mtype == "ERR":
                    self._log("ERROR: " + payload)
                elif mtype == "SENT":
                    self._log(">> " + payload)
                elif mtype == "LINE":
                    line = payload
                    st = parse_tray_status_line(line)
                    if st:
                        self._tray_status_latest = st
                        self._tray_status_ts = time.time()
                        for k, var in self.tray_status_vars.items():
                            if k in st:
                                var.set(st[k])

                    # Optional: stop auto mode if tray prints an obvious "no more cells" message
                    u = line.upper()
                    if any(tok in u for tok in ("NO_MORE", "NO MORE", "EMPTY", "OUT OF CELLS", "END")):
                        if self.auto_next_enabled.get():
                            self.auto_next_enabled.set(False)
                            self._auto_next_in_progress = False
                            self._low_current_since = None
                            self._log("AUTO: Tray reports no more cells -> auto mode stopped")
        except queue.Empty:
            pass
        self.after(50, self._pump_tray_rx)

    # -------- PSU actions --------
    def _psu_connect(self):
        port = self.psu_port.get().strip()
        if not port:
            messagebox.showwarning("PSU", "Select a PSU serial port.")
            return
        try:
            baud = int(self.psu_baud.get().strip())
            slave = int(self.psu_slave.get().strip())
        except ValueError:
            messagebox.showwarning("PSU", "PSU baud and ID must be numbers.")
            return

        self.dps = DPS5020(port, baudrate=baud, slave_id=slave)
        if self.dps.connect():
            self.psu_conn_var.set(f"PSU: Connected {port} @ {baud}, ID {slave}")
            self._log(f"PSU connected: {port} (baud {baud}, id {slave})")
        else:
            self.psu_conn_var.set("PSU: Connect failed")
            self._log("ERROR: PSU connect failed")
            self.dps = None

    def _psu_disconnect(self):
        if self.dps:
            self.dps.disconnect()
        self.dps = None
        self.psu_conn_var.set("PSU: Disconnected")
        self.psu_out_var.set("--")
        self.psu_current_var.set("--.- A")
        self._log("PSU disconnected")

    def _psu_on(self):
        if not self.dps or not self.dps.connected:
            self._log("ERROR: PSU not connected")
            return
        ok = self.dps.set_output_enabled(True)
        self._log("PSU ON" if ok else "ERROR: PSU ON failed")

    def _psu_off(self):
        if not self.dps or not self.dps.connected:
            self._log("ERROR: PSU not connected")
            return
        ok = self.dps.set_output_enabled(False)
        self._log("PSU OFF" if ok else "ERROR: PSU OFF failed")

    def _psu_toggle(self):
        if not self.dps or not self.dps.connected:
            self._log("ERROR: PSU not connected")
            return
        ok, new_state = self.dps.toggle_output()
        if ok and new_state is not None:
            self._log(f"PSU TOGGLE -> {'ON' if new_state else 'OFF'}")
        else:
            self._log("ERROR: PSU toggle failed")

    def _poll_psu(self):
        cur_a = None
        out_en = None

        if self.dps and self.dps.connected:
            out_en = self.dps.read_output_enabled()
            cur_a = self.dps.read_current_a()

            self.psu_out_var.set("--" if out_en is None else ("ON" if out_en else "OFF"))
            self.psu_current_var.set("--.- A" if cur_a is None else f"{cur_a:.2f} A")

        # --- AUTO NEXT LOOP WITH 10s STABILIZE ---
        if self.auto_next_enabled.get():
            if (cur_a is not None) and (out_en == 1):
                now = time.time()
                if cur_a <= self.auto_next_threshold_a:
                    if self._low_current_since is None:
                        self._low_current_since = now
                        self._log("AUTO: current low, waiting 10s to stabilize...")

                    low_for = now - self._low_current_since
                    if (low_for >= self.auto_next_stabilize_s) and (not self._auto_next_in_progress):
                        # reset timer so we don't instantly re-trigger
                        self._low_current_since = None
                        self._auto_advance_step()
                else:
                    self._low_current_since = None
            else:
                self._low_current_since = None
        else:
            self._auto_next_in_progress = False
            self._low_current_since = None

        self.after(400, self._poll_psu)

    # -------- PX100 actions (REPLACED WITH YOUR DRIVER) --------
    def _px_connect(self):
        port = self.px_port.get().strip()
        if not port:
            messagebox.showwarning("PX100", "Select a PX100 serial port.")
            return
        try:
            baud = int(self.px_baud.get().strip())
        except ValueError:
            messagebox.showwarning("PX100", "PX100 baud must be a number.")
            return

        self.px100 = PX100Controller(port, baud=baud, timeout_s=2.0)
        if self.px100.connect():
            self.px_conn_var.set(f"PX100: Connected {port} @ {baud}")
            ok = self.px100.probe()
            self._log("PX100 probe OK" if ok else "PX100 connected but not responding (wrong port/wiring)")
        else:
            self.px_conn_var.set("PX100: Connect failed")
            self._log("ERROR: PX100 connect failed")
            self.px100 = None

    def _px_disconnect(self):
        if self.px100:
            try:
                self.px100.set_load_enabled(False)
            except Exception:
                pass
            self.px100.disconnect()
        self.px100 = None
        self.px_conn_var.set("PX100: Disconnected")
        self._log("PX100 disconnected")

    def _px_start(self):
        if not self.px100 or not self.px100.connected:
            self._log("ERROR: PX100 not connected")
            return
        try:
            current_a = float(self.px_set_current.get().strip())
            cutoff_v = float(self.px_set_cutoff.get().strip())
        except ValueError:
            messagebox.showwarning("PX100", "Enter numeric current and cutoff voltage.")
            return

        ok = self.px100.start_capacity_test(current_a=current_a, cutoff_v=cutoff_v, timer_s=0)
        self._log("PX100 START TEST" if ok else "ERROR: PX100 start failed")

    def _px_reset(self):
        if not self.px100 or not self.px100.connected:
            self._log("ERROR: PX100 not connected")
            return
        ok = self.px100.reset()
        self._log("PX100 RESET" if ok else "ERROR: PX100 reset failed")

    def _px_stop(self):
        if not self.px100 or not self.px100.connected:
            self._log("ERROR: PX100 not connected")
            return
        ok = self.px100.set_load_enabled(False)
        self._log("PX100 STOP LOAD" if ok else "ERROR: PX100 stop failed")

    def _poll_px100(self):
        if self.px100 and self.px100.connected:
            # readAll each poll (simple and robust)
            row = self.px100.read_all()

            # voltage/current/temp
            v = safe_float(row.get("voltage"))
            i = safe_float(row.get("current"))
            temp = safe_float(row.get("temp"))

            self.px_v_var.set("--.- V" if v is None else f"{v:.3f} V")
            self.px_i_var.set("--.- A" if i is None else f"{i:.3f} A")
            self.px_temp_var.set("-- °C" if temp is None else f"{temp:.1f} °C")

            # capacity/energy: prefer Ah/Wh fields, fall back to mAh/mWh if your driver returns those
            cap_ah = safe_float(row.get("cap_ah"))
            cap_wh = safe_float(row.get("cap_wh"))
            cap_mah = safe_float(row.get("cap_mah"))
            cap_mwh = safe_float(row.get("cap_mwh"))

            if cap_mah is None and cap_ah is not None:
                cap_mah = cap_ah * 1000.0
            if cap_mwh is None and cap_wh is not None:
                cap_mwh = cap_wh * 1000.0

            self.px_mah_var.set("---- mAh" if cap_mah is None else f"{cap_mah:.0f} mAh")
            self.px_mwh_var.set("---- mWh" if cap_mwh is None else f"{cap_mwh:.0f} mWh")

            # time: could be seconds, or "HH:MM:SS", or a datetime.time-like object
            tval = row.get("time")
            tstr = None
            if isinstance(tval, (int, float)):
                sec = int(max(0, tval))
                h = sec // 3600
                m = (sec % 3600) // 60
                s = sec % 60
                tstr = f"{h:02d}:{m:02d}:{s:02d}"
            elif isinstance(tval, str):
                tstr = tval
            else:
                # last resort: stringify if it looks meaningful
                if tval is not None:
                    tstr = str(tval)

            self.px_time_var.set("--:--:--" if not tstr else tstr)

        self.after(400, self._poll_px100)


if __name__ == "__main__":
    app = App()
    app.mainloop()
