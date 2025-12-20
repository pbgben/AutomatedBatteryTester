#!/usr/bin/env python3
import glob
import math
import os
import queue
import threading
import time
from datetime import datetime
from escpos.printer import Network

import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# ---- PX100 driver (your working codebase) ----
# Run from your repo root (or ensure these are on PYTHONPATH)
from instruments.instrument import Instrument
from instruments.px100 import PX100 as PX100_Driver

# ---- Graphing ----
# pip install matplotlib
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


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


def safe_float(x):
    try:
        x = float(x)
        if math.isnan(x) or math.isinf(x):
            return None
        return x
    except Exception:
        return None


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
    # Your original mapping (may vary by firmware):
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


# -------------------- PX100 driver adapter + controller --------------------
class SerialRawAdapter:
    """Adapter for PX100 driver transport expectations."""
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


def try_command(dev: PX100_Driver, cmd_name: str, value):
    """Call the driver's command method in a couple common forms."""
    try:
        return dev.command(cmd_name, value)
    except TypeError:
        pass
    try:
        return dev.command({cmd_name: value})
    except TypeError:
        pass
    raise


def normalize_readall(data) -> dict:
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


class PX100Controller:
    """Thin wrapper used by the GUI."""
    def __init__(self, port: str, baud: int = 9600, timeout_s: float = 2.0):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.transport = None
        self.dev = None
        self.connected = False
        self.lock = threading.Lock()

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
        return bool(self.read_all())

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

    def set_load_enabled(self, enabled: bool) -> bool:
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_ENABLE, bool(enabled))
                return True
            except Exception:
                return False

    def _set_if_needed(self, cmd_name: str, current_val, desired_val, tol: float) -> bool:
        with self.lock:
            if not self.connected or not self.dev:
                return False
            cur = safe_float(current_val)
            des = safe_float(desired_val)
            if cur is None or des is None:
                try:
                    try_command(self.dev, cmd_name, desired_val)
                    return True
                except Exception:
                    return False
            if abs(cur - des) <= tol:
                return True
            try:
                try_command(self.dev, cmd_name, desired_val)
                return True
            except Exception:
                return False

    def set_current_a(self, current_a: float) -> bool:
        st = self.read_all()
        return self._set_if_needed(
            Instrument.COMMAND_SET_CURRENT,
            current_val=st.get("set_current"),
            desired_val=float(current_a),
            tol=self.tol_a,
        )

    def set_cutoff_v(self, cutoff_v: float) -> bool:
        st = self.read_all()
        return self._set_if_needed(
            Instrument.COMMAND_SET_VOLTAGE,
            current_val=st.get("set_voltage"),
            desired_val=float(cutoff_v),
            tol=self.tol_v,
        )

    def start_capacity_test(self, current_a: float, cutoff_v: float) -> bool:
        ok = self.set_load_enabled(False)
        time.sleep(0.15)
        ok = ok and self.set_current_a(current_a)
        time.sleep(0.10)
        ok = ok and self.set_cutoff_v(cutoff_v)
        time.sleep(0.10)
        ok = ok and self.set_load_enabled(True)
        return ok


# -------------------- GUI + automation --------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Cell Tray + DPS5020 + PX100")
        self.geometry("1200x760")

        # Tray
        self.tray_rx = queue.Queue()
        self.tray_tx = queue.Queue()
        self.tray = TraySerialWorker(self.tray_rx, self.tray_tx)
        self.tray.start()

        # PSU + Load
        self.dps = None
        self.px100 = None

        # PSU startup grace (avoids "unexpected OFF" right after enabling output)
        self.psu_on_grace_s = 4.0     # time allowed for PSU to report ON
        self._psu_on_cmd_ts = None    # timestamp when we commanded PSU ON

        # Receipt printer
        self.receipt_printer_ip = "192.168.14.223"
        self.receipt_printer_port = 9100  # common raw TCP port for receipt printers :contentReference[oaicite:2]{index=2}
        self.receipt_dots = 384  # image width in pixels for printing


        # ---- last PSU readings (for automation) ----
        self._psu_last_cur_a = None
        self._psu_last_out_en = None

        # ---- Tray status snapshot (for automation) ----
        self._tray_status_latest = {}
        self._tray_status_ts = 0.0
        self._tray_empty_hint = False

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

        # test settings
        self.px_set_current = tk.StringVar(value="1.00")
        self.px_set_cutoff = tk.StringVar(value="3.00")

        # status readouts
        self.px_v_var = tk.StringVar(value="--.- V")
        self.px_i_var = tk.StringVar(value="--.- A")
        self.px_mah_var = tk.StringVar(value="---- mAh")
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

        # ---------------- automation checkboxes ----------------
        self.auto_charge = tk.BooleanVar(value=True)   # charge until cutoff
        self.auto_test = tk.BooleanVar(value=True)     # capacity test
        self.auto_run = tk.BooleanVar(value=False)     # keep going cell-to-cell

        # charge cutoff behavior
        self.charge_cutoff_a = 0.10
        self.charge_stabilize_s = 10.0
        self._charge_low_since = None

        # process state
        self._proc_state = "IDLE"
        self._proc_cancel = False

        # PX100 test logging for graph
        self._test_running = False
        self._test_t0 = None
        self._test_samples = []   # list of {"t":..., "v":..., "mah":...}
        self._test_cutoff_v = None
        self._last_saved_path = None

        # Graph window
        self._graph_win = None
        self._graph_canvas = None
        self._graph_fig = None
        self._graph_ax_v = None
        self._graph_ax_mah = None

        self._build_ui()
        self._refresh_ports()

        self.after(50, self._pump_tray_rx)
        self.after(400, self._poll_psu)
        self.after(400, self._poll_px100)

    # ---------------- UI ----------------
    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("TButton", font=("Sans", 18))
        style.configure("TLabel", font=("Sans", 14))
        style.configure("Conn.TLabel", font=("Sans", 11))
        style.configure("Conn.TButton", font=("Sans", 11))

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

        # NEXT CELL now triggers the automation sequence for the next cell
        ttk.Button(tray_ctrl, text="NEXT CELL (run cycle)", command=self._start_cycle_next_cell).pack(fill="x", pady=5)

        ttk.Button(tray_ctrl, text="UNLOAD", command=lambda: self._tray_send("UNLOAD")).pack(fill="x", pady=5)
        ttk.Separator(tray_ctrl).pack(fill="x", pady=10)
        ttk.Button(tray_ctrl, text="STOP (cancel cycle)", command=self._stop_cycle).pack(fill="x", pady=5)

        # --- Automation options ---
        auto_ctrl = ttk.LabelFrame(left, text="Automation Options", padding=10)
        auto_ctrl.pack(fill="x", pady=(10, 0))

        ttk.Checkbutton(auto_ctrl, text="Charge cell (PSU until 0.10A for 10s)", variable=self.auto_charge).pack(anchor="w")
        ttk.Checkbutton(auto_ctrl, text="Capacity test (PX100)", variable=self.auto_test).pack(anchor="w")
        ttk.Checkbutton(auto_ctrl, text="Auto-run all cells (repeat until tray empty)", variable=self.auto_run).pack(anchor="w", pady=(6, 0))
        ttk.Button(auto_ctrl, text="TEST PRINTER", command=self._printer_test).pack(fill="x", pady=(8, 0))


        # --- PSU controls ---
        psu_ctrl = ttk.LabelFrame(left, text="PSU Controls", padding=10)
        psu_ctrl.pack(fill="x", pady=(10, 0))

        psu_btns = ttk.Frame(psu_ctrl)
        psu_btns.pack(fill="x")
        ttk.Button(psu_btns, text="PSU ON", command=self._psu_on).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ttk.Button(psu_btns, text="PSU OFF", command=self._psu_off).pack(side="left", expand=True, fill="x", padx=(0, 6))

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
        ttk.Button(px_btns, text="START TEST", command=self._px_start).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ttk.Button(px_btns, text="RESET", command=self._px_reset).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ttk.Button(px_btns, text="STOP LOAD", command=self._px_stop).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ttk.Button(px_btns, text="SHOW GRAPH", command=self._px_show_graph).pack(side="left", expand=True, fill="x")

        # --- Status panel ---
        status = ttk.LabelFrame(right, text="Status (auto update)", padding=10)
        status.pack(fill="x")

        grid = ttk.Frame(status)
        grid.pack(fill="x")

        def srow(r, label, var):
            ttk.Label(grid, text=label).grid(row=r, column=0, sticky="w", padx=(0, 10), pady=5)
            ttk.Label(grid, textvariable=var, font=("Sans", 16, "bold")).grid(row=r, column=1, sticky="w", pady=5)

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
        srow(11, "PX100 time:", self.px_time_var)
        srow(12, "PX100 temp:", self.px_temp_var)

        # --- Log ---
        logf = ttk.LabelFrame(right, text="Log", padding=10)
        logf.pack(fill="both", expand=True, pady=(10, 0))

        self.log = tk.Text(logf, height=12, wrap="word", font=("Sans", 12))
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


    def _print_receipt_with_graph(self, graph_png_path: str, capacity_mah):
        """
        Print a receipt: title + timestamp + capacity + the graph image, then cut.
        Assumes an ESC/POS compatible network receipt printer.
        """
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        try:
            p = Network(self.receipt_printer_ip, port=self.receipt_printer_port, timeout=10)

            p.set(align="center", bold=True, width=2, height=2)
            p.text("CELL CAPACITY\n")

            p.set(align="center", bold=False, width=1, height=1)
            p.text(f"{ts}\n")

            if capacity_mah is None:
                p.text("Capacity: (unknown)\n")
            else:
                p.set(bold=True)
                p.text(f"Capacity: {float(capacity_mah):.0f} mAh\n")
                p.set(bold=False)

            p.text("\n")

            # Print the graph image
            # python-escpos supports images on network printers :contentReference[oaicite:3]{index=3}
            p.set(align="center")
            p.image(graph_png_path)

            p.text("\n")
            p.cut()

        except Exception as e:
            self._log(f"CYCLE: ERROR printing receipt: {e}")

    def _printer_test(self):
        """
        Print a quick test receipt to the configured network printer.
        Prints text + a small generated test image so we verify image printing too.
        """
        try:
            from PIL import Image, ImageDraw, ImageFont
        except Exception as e:
            messagebox.showerror("Printer test", f"Pillow not available:\n{e}")
            return

        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Make a tiny test image (works on most ESC/POS printers)
        img_w, img_h = int(self.receipt_dots), 140
        img = Image.new("RGB", (img_w, img_h), "white")
        d = ImageDraw.Draw(img)

        d.rectangle([0, 0, img_w - 1, img_h - 1], outline="black", width=2)
        d.text((10, 10), "PRINTER TEST", fill="black")
        d.text((10, 40), ts, fill="black")
        d.text((10, 70), f"IP: {self.receipt_printer_ip}", fill="black")

        # Save to a temp file (python-escpos image() accepts a file path reliably)
        tmp_path = os.path.join(os.getcwd(), "printer_test.png")
        try:
            img.save(tmp_path)
        except Exception as e:
            messagebox.showerror("Printer test", f"Failed to save temp test image:\n{e}")
            return

        try:
            p = Network(self.receipt_printer_ip, port=self.receipt_printer_port, timeout=10)

            p.set(align="center", bold=True, width=2, height=2)
            p.text("TEST RECEIPT\n")
            p.set(align="center", bold=False, width=1, height=1)
            p.text(f"{ts}\n")
            p.text("\n")

            p.set(align="center")
            p.image(tmp_path)

            p.text("\n")
            p.cut()

            self._log(f"Printer test sent to {self.receipt_printer_ip}:{self.receipt_printer_port}")

        except Exception as e:
            messagebox.showerror(
                "Printer test",
                f"Failed to print to {self.receipt_printer_ip}:{self.receipt_printer_port}\n\n{e}",
            )
            self._log(f"ERROR: printer test failed: {e}")
        finally:
            try:
                os.remove(tmp_path)
            except Exception:
                pass


    # ---------------- common helpers ----------------
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

    def _tray_status_fresh(self, max_age_s: float = 2.0) -> bool:
        return (time.time() - self._tray_status_ts) <= max_age_s

    def _tray_cell_looks_loaded(self) -> bool:
        """Best-effort: loaded if any of contacts/aligned/detectClosed == 1 in STATUS."""
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

    # ---------------- Tray actions ----------------
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

                    # Best-effort "empty tray" hints from free text
                    u = line.upper()
                    if any(tok in u for tok in ("NO_MORE", "NO MORE", "EMPTY", "OUT OF CELLS", "END")):
                        self._tray_empty_hint = True
        except queue.Empty:
            pass
        self.after(50, self._pump_tray_rx)

    # ---------------- PSU actions ----------------
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

    def _poll_psu(self):
        if self.dps and self.dps.connected:
            out_en = self.dps.read_output_enabled()
            cur_a = self.dps.read_current_a()

            self._psu_last_out_en = out_en
            self._psu_last_cur_a = cur_a

            self.psu_out_var.set("--" if out_en is None else ("ON" if out_en else "OFF"))
            self.psu_current_var.set("--.- A" if cur_a is None else f"{cur_a:.2f} A")
        else:
            self._psu_last_out_en = None
            self._psu_last_cur_a = None

        self.after(400, self._poll_psu)

    # ---------------- PX100 actions ----------------
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
        self._test_running = False
        if self.px100:
            try:
                self.px100.set_load_enabled(False)
            except Exception:
                pass
            self.px100.disconnect()
        self.px100 = None
        self.px_conn_var.set("PX100: Disconnected")
        self._log("PX100 disconnected")

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
        self._test_running = False
        self._log("PX100 STOP LOAD" if ok else "ERROR: PX100 stop failed")

    def _px_start(self):
        """Manual start (kept), also used by automation."""
        if not self.px100 or not self.px100.connected:
            self._log("ERROR: PX100 not connected")
            return False
        try:
            current_a = float(self.px_set_current.get().strip())
            cutoff_v = float(self.px_set_cutoff.get().strip())
        except ValueError:
            messagebox.showwarning("PX100", "Enter numeric current and cutoff voltage.")
            return False

        # Reset capacity tester before test
        self._log("PX100: Resetting before test…")
        self.px100.reset()
        time.sleep(0.2)

        # Start logging
        self._test_samples = []
        self._test_t0 = time.time()
        self._test_running = True
        self._test_cutoff_v = cutoff_v

        ok = self.px100.start_capacity_test(current_a=current_a, cutoff_v=cutoff_v)
        self._log("PX100 START TEST" if ok else "ERROR: PX100 start failed")
        if not ok:
            self._test_running = False
        return ok

    def _px_extract_mah(self, row: dict):
        cap_mah = safe_float(row.get("cap_mah"))
        if cap_mah is not None:
            return cap_mah
        cap_ah = safe_float(row.get("cap_ah"))
        if cap_ah is not None:
            return cap_ah * 1000.0
        # fallbacks if driver uses different keys
        for k in ("capacity_mah", "mah"):
            v = safe_float(row.get(k))
            if v is not None:
                return v
        for k in ("capacity_ah", "ah"):
            v = safe_float(row.get(k))
            if v is not None:
                return v * 1000.0
        return None

    def _px_record_sample(self, row: dict):
        if not self._test_running or self._test_t0 is None:
            return
        v = safe_float(row.get("voltage"))
        mah = self._px_extract_mah(row)
        if v is None and mah is None:
            return
        t = time.time() - self._test_t0
        self._test_samples.append({"t": t, "v": v, "mah": mah})

    def _poll_px100(self):
        if self.px100 and self.px100.connected:
            row = self.px100.read_all()

            # show on UI
            v = safe_float(row.get("voltage"))
            i = safe_float(row.get("current"))
            temp = safe_float(row.get("temp"))
            mah = self._px_extract_mah(row)

            self.px_v_var.set("--.- V" if v is None else f"{v:.3f} V")
            self.px_i_var.set("--.- A" if i is None else f"{i:.3f} A")
            self.px_temp_var.set("-- °C" if temp is None else f"{temp:.1f} °C")
            self.px_mah_var.set("---- mAh" if mah is None else f"{mah:.0f} mAh")

            # time formatting (best-effort)
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
            elif tval is not None:
                tstr = str(tval)
            self.px_time_var.set("--:--:--" if not tstr else tstr)

            # record samples for graph
            if self._test_running:
                self._px_record_sample(row)

                # stop condition by cutoff voltage (same logic as your script)
                if self._test_cutoff_v is not None and v is not None and v <= float(self._test_cutoff_v):
                    self._log(f"PX100: Cutoff reached (V={v:.3f} <= {self._test_cutoff_v:.3f}). Stopping test.")
                    try:
                        self.px100.set_load_enabled(False)
                    except Exception:
                        pass
                    self._test_running = False
                    self._on_capacity_test_finished()

        self.after(400, self._poll_px100)

    # ---------------- Graph window + saving ----------------
    def _px_show_graph(self):
        if not self._test_samples:
            messagebox.showinfo("PX100 Graph", "No samples yet.")
            return

        # Reuse existing window if open
        if self._graph_win and self._graph_win.winfo_exists():
            self._px_update_graph()
            self._graph_win.lift()
            return

        win = tk.Toplevel(self)
        win.title("PX100 Capacity Test Graph")
        win.geometry("950x650")
        self._graph_win = win

        fig = Figure(dpi=100)
        ax_v = fig.add_subplot(111)
        ax_mah = ax_v.twinx()

        self._graph_fig = fig
        self._graph_ax_v = ax_v
        self._graph_ax_mah = ax_mah

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._graph_canvas = canvas

        toolbar = NavigationToolbar2Tk(canvas, win)
        toolbar.update()

        btn_row = ttk.Frame(win, padding=6)
        btn_row.pack(fill="x")
        ttk.Button(btn_row, text="Refresh", command=self._px_update_graph).pack(side="left")
        ttk.Button(btn_row, text="Save PNG (manual)", command=self._px_save_graph_png_dialog).pack(side="left", padx=(6, 0))
        ttk.Button(btn_row, text="Close", command=win.destroy).pack(side="right")

        self._px_update_graph()

    def _px_update_graph(self):
        if not (self._graph_win and self._graph_win.winfo_exists()):
            return
        if not self._test_samples:
            return

        ax_v = self._graph_ax_v
        ax_mah = self._graph_ax_mah
        fig = self._graph_fig

        ax_v.cla()
        ax_mah.cla()

        x_v = [s["t"] for s in self._test_samples if s.get("v") is not None and s.get("t") is not None]
        y_v = [s["v"] for s in self._test_samples if s.get("v") is not None and s.get("t") is not None]
        if x_v:
            ax_v.plot(x_v, y_v)
        ax_v.set_xlabel("Time (s)")
        ax_v.set_ylabel("Voltage (V)")

        x_m = [s["t"] for s in self._test_samples if s.get("mah") is not None and s.get("t") is not None]
        y_m = [s["mah"] for s in self._test_samples if s.get("mah") is not None and s.get("t") is not None]
        if x_m:
            ax_mah.plot(x_m, y_m)
        ax_mah.set_ylabel("Capacity (mAh)")

        fig.tight_layout()
        self._graph_canvas.draw()

    def _save_graph_png_auto(self, folder: str, capacity_mah):
        """
        Save graph automatically with timestamp + capacity in filename.
        Returns saved path or None.
        """
        os.makedirs(folder, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        cap_str = "unknown"
        if capacity_mah is not None:
            cap_str = f"{capacity_mah:.0f}mAh"
        filename = f"capacity_{cap_str}_{ts}.png"
        path = os.path.join(folder, filename)

        # Build a fresh figure for saving (works even if graph window isn't open)
        dpi = 200
        width_in = self.receipt_dots / dpi     # e.g. 384px / 200dpi = 1.92"
        height_in = 4.2                        # tune this if you want shorter/longer receipts
        fig = Figure(dpi=dpi, figsize=(width_in, height_in))

        ax_v = fig.add_subplot(111)
        ax_mah = ax_v.twinx()

        x_v = [s["t"] for s in self._test_samples if s.get("v") is not None and s.get("t") is not None]
        y_v = [s["v"] for s in self._test_samples if s.get("v") is not None and s.get("t") is not None]
        if x_v:
            ax_v.plot(x_v, y_v)
        ax_v.set_xlabel("Time (s)")
        ax_v.set_ylabel("Voltage (V)")

        x_m = [s["t"] for s in self._test_samples if s.get("mah") is not None and s.get("t") is not None]
        y_m = [s["mah"] for s in self._test_samples if s.get("mah") is not None and s.get("t") is not None]
        if x_m:
            ax_mah.plot(x_m, y_m)
        ax_mah.set_ylabel("Capacity (mAh)")

        fig.tight_layout()
        fig.savefig(path)
        return path

    def _px_save_graph_png_dialog(self):
        # Keep a manual option if you want it
        try:
            from tkinter import filedialog
        except Exception:
            messagebox.showerror("Save graph", "filedialog not available")
            return

        if not self._test_samples:
            messagebox.showwarning("Save graph", "No samples to save.")
            return

        path = filedialog.asksaveasfilename(
            title="Save graph as PNG",
            defaultextension=".png",
            filetypes=[("PNG image", "*.png")],
            initialfile="capacity_test.png",
        )
        if not path:
            return

        # Save via the auto saver but to chosen path
        try:
            folder = os.path.dirname(path) or "."
            base = os.path.basename(path)
            os.makedirs(folder, exist_ok=True)

            # Build and save a fresh figure
            fig = Figure(dpi=200)
            ax_v = fig.add_subplot(111)
            ax_mah = ax_v.twinx()

            x_v = [s["t"] for s in self._test_samples if s.get("v") is not None and s.get("t") is not None]
            y_v = [s["v"] for s in self._test_samples if s.get("v") is not None and s.get("t") is not None]
            if x_v:
                ax_v.plot(x_v, y_v)
            ax_v.set_xlabel("Time (s)")
            ax_v.set_ylabel("Voltage (V)")

            x_m = [s["t"] for s in self._test_samples if s.get("mah") is not None and s.get("t") is not None]
            y_m = [s["mah"] for s in self._test_samples if s.get("mah") is not None and s.get("t") is not None]
            if x_m:
                ax_mah.plot(x_m, y_m)
            ax_mah.set_ylabel("Capacity (mAh)")

            fig.tight_layout()
            fig.savefig(path)
            self._log(f"Saved graph: {path}")
        except Exception as e:
            messagebox.showerror("Save graph", f"Failed to save:\n{e}")

    # ---------------- Automation cycle ----------------
    def _stop_cycle(self):
        self._proc_cancel = True
        self._proc_state = "IDLE"
        self._charge_low_since = None
        self._test_running = False
        self._log("CYCLE: cancelled")
        # safety: turn off PSU + PX100 load
        try:
            if self.dps and self.dps.connected:
                self.dps.set_output_enabled(False)
        except Exception:
            pass
        try:
            if self.px100 and self.px100.connected:
                self.px100.set_load_enabled(False)
        except Exception:
            pass

    def _start_cycle_next_cell(self):
        """
        User flow:
          - Put tray in, click LOAD (HOME+RECONNECT)
          - Tick Charge and/or Test, optionally Auto-run
          - Click NEXT CELL (run cycle) -> does:
            NEXT -> RECONNECT -> CHARGE (optional) -> PSU OFF -> PX100 reset+test (optional) -> graph+save
            If Auto-run checked: repeats NEXT until tray empty / failure / cancelled.
        """
        if self._proc_state != "IDLE":
            self._log("CYCLE: already running")
            return

        if not self.tray.is_connected():
            self._log("ERROR: Tray not connected")
            return

        if self.auto_charge.get() and (not self.dps or not self.dps.connected):
            self._log("ERROR: Charge enabled but PSU not connected")
            return

        if self.auto_test.get() and (not self.px100 or not self.px100.connected):
            self._log("ERROR: Test enabled but PX100 not connected")
            return

        self._proc_cancel = False
        self._tray_empty_hint = False
        self._proc_state = "MOVE_NEXT"
        self._log("CYCLE: NEXT cell…")
        self._tray_send("NEXT")
        self.after(250, lambda: self._tray_send("RECONNECT"))
        self.after(300, self._cycle_tick)

    def _cycle_tick(self):
        if self._proc_cancel:
            self._proc_state = "IDLE"
            return

        # stop if tray hints empty
        if self._tray_empty_hint:
            self._log("CYCLE: tray reported empty / no more cells. Stopping.")
            self._proc_state = "IDLE"
            return

        if self._proc_state == "MOVE_NEXT":
            # wait for a cell to be "loaded"
            self._proc_state = "WAIT_CELL"
            self._wait_cell_start = time.time()
            self.after(200, self._cycle_tick)
            return

        if self._proc_state == "WAIT_CELL":
            if self._tray_cell_looks_loaded():
                self._log("CYCLE: cell loaded.")
                # decide next phase
                if self.auto_charge.get():
                    self._proc_state = "CHARGE_START"
                elif self.auto_test.get():
                    self._proc_state = "TEST_START"
                else:
                    self._proc_state = "FINISH"
                self.after(100, self._cycle_tick)
                return

            # timeout -> likely empty tray or jam
            if (time.time() - self._wait_cell_start) > 6.0:
                self._log("CYCLE: could not confirm a cell (empty tray or jam). Stopping.")
                self._proc_state = "IDLE"
                return

            self.after(200, self._cycle_tick)
            return

        if self._proc_state == "CHARGE_START":
            self._log("CYCLE: charging (PSU ON)…")
            self._charge_low_since = None
            self._psu_on_cmd_ts = time.time()

            ok = self.dps.set_output_enabled(True)
            if not ok:
                self._log("CYCLE: ERROR turning PSU ON. Stopping.")
                self._proc_state = "IDLE"
                return

            self._proc_state = "CHARGE_WAIT"
            self.after(400, self._cycle_tick)
            return

        if self._proc_state == "CHARGE_WAIT":
            cur = self._psu_last_cur_a
            out_en = self._psu_last_out_en
            now = time.time()

            # --- Grace period: allow the PSU a few seconds to actually report ON ---
            if self._psu_on_cmd_ts is not None:
                if (now - self._psu_on_cmd_ts) < self.psu_on_grace_s:
                    # During grace period we don't treat OFF as an error
                    self.after(400, self._cycle_tick)
                    return

            # After grace: if PSU still says OFF, then it's a real problem
            if out_en == 0:
                self._log("CYCLE: PSU output is OFF unexpectedly (after grace). Stopping.")
                self._proc_state = "IDLE"
                return

            # If we can't read current yet, just keep waiting
            if cur is None:
                self.after(400, self._cycle_tick)
                return

            # Charge complete when current <= 0.10A for 10 seconds
            if cur <= self.charge_cutoff_a:
                if self._charge_low_since is None:
                    self._charge_low_since = now
                    self._log("CYCLE: current low, waiting 10s stabilize…")
                low_for = now - self._charge_low_since
                if low_for >= self.charge_stabilize_s:
                    self._log("CYCLE: charge complete (cutoff stable). PSU OFF.")
                    self.dps.set_output_enabled(False)
                    self._charge_low_since = None
                    self._psu_on_cmd_ts = None

                    self._proc_state = "TEST_START" if self.auto_test.get() else "FINISH"
                    self.after(200, self._cycle_tick)
                    return
            else:
                self._charge_low_since = None

            self.after(400, self._cycle_tick)
            return


        if self._proc_state == "TEST_START":
            # ensure PSU off
            try:
                if self.dps and self.dps.connected:
                    self.dps.set_output_enabled(False)
            except Exception:
                pass

            self._log("CYCLE: starting capacity test…")
            ok = self._px_start()
            if not ok:
                self._log("CYCLE: ERROR starting PX100 test. Stopping.")
                self._proc_state = "IDLE"
                return
            self._proc_state = "TEST_WAIT"
            self.after(500, self._cycle_tick)
            return

        if self._proc_state == "TEST_WAIT":
            # Test completion is detected in _poll_px100 via cutoff and calls _on_capacity_test_finished()
            # So here we just keep waiting.
            if self._test_running:
                self.after(500, self._cycle_tick)
                return
            # If _test_running is false but we didn't transition yet, we'll finish here.
            self._proc_state = "FINISH"
            self.after(100, self._cycle_tick)
            return

        if self._proc_state == "FINISH":
            self._log("CYCLE: finished cell.")

            # Auto-run next cell?
            if self.auto_run.get():
                self._log("CYCLE: auto-run -> next cell…")
                self._proc_state = "MOVE_NEXT"
                self._tray_empty_hint = False
                self._tray_send("NEXT")
                self.after(250, lambda: self._tray_send("RECONNECT"))
                self.after(300, self._cycle_tick)
                return

            self._proc_state = "IDLE"
            return

    def _on_capacity_test_finished(self):
        """
        Called when the PX100 test hits cutoff and stops.
        Saves graph automatically with timestamp+capacity and prints a receipt containing the graph.
        """
        # compute final capacity
        cap = None
        for s in reversed(self._test_samples):
            cap = safe_float(s.get("mah"))
            if cap is not None:
                break

        folder = os.path.join(os.getcwd(), "capacity_graphs")
        path = None
        try:
            path = self._save_graph_png_auto(folder=folder, capacity_mah=cap)
            self._last_saved_path = path
            if path:
                self._log(f"CYCLE: saved graph -> {path}")
        except Exception as e:
            self._log(f"CYCLE: ERROR saving graph: {e}")

        # Print receipt (instead of showing graph window)
        if path:
            self._log("CYCLE: printing receipt…")
            self._print_receipt_with_graph(path, cap)

        # proceed in the automation state machine
        if self._proc_state == "TEST_WAIT":
            self._proc_state = "FINISH"
            self.after(100, self._cycle_tick)


if __name__ == "__main__":
    app = App()
    app.mainloop()
