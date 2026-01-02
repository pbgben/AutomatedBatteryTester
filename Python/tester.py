#!/usr/bin/env python3
"""
Cell Tray + DPS5020 + PX100 automation GUI (single-file)

What it does
- Tray control (Arduino serial): LOAD (HOME+RECONNECT), NEXT cycle, UNLOAD, STOP
- PSU control (DPS5020 over Modbus RTU): ON/OFF + current polling
- PX100 capacity test using your working driver (instruments.*)
- Automation checkboxes:
    [x] Charge cell (PSU ON until <=0.10A for 10s, then PSU OFF)
    [x] Capacity test (PX100 reset + discharge until cutoff V)
    [x] Auto-run all cells (repeat NEXT until tray empty / can't confirm cell)
- Safety:
    - PSU is forced OFF before starting any capacity test (even if charge is unticked)
    - PSU startup grace avoids false "unexpected OFF" right after turning ON
    - Tray contacts are only CLOSED during: voltage-check, charge, discharge
      and opened immediately afterwards to prevent parasitic draw.
- Receipt printing (network ESC/POS):
    - Prints: timestamp, capacity, test current, cutoff voltage, charge/discharge durations
    - Prints a voltage curve graph (no capacity ramp line)
    - Also prints a SKIPPED receipt when a cell is skipped (e.g., low voltage)

Requirements
- pip: pyserial pymodbus python-escpos pillow matplotlib
- apt (Ubuntu): python3-tk python3-pil.imagetk
- Your repo must provide: instruments.instrument and instruments.px100
"""

import glob
import math
import os
import queue
import threading
import time
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# ---- PX100 driver (your working codebase) ----
from instruments.instrument import Instrument
from instruments.px100 import PX100 as PX100_Driver
from devices.ports import auto_detect_ports

# ---- Printing ----
from escpos.printer import Network
from PIL import Image  # for thumbnailing and image prep

# ---- Plotting (save PNGs only; no GUI graph window) ----
import matplotlib
matplotlib.use("TkAgg")  # Tkinter GUI + save-to-file rendering
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# -------------------- Port helpers --------------------
def list_serial_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for dev in glob.glob(pat):
            if dev not in ports:
                ports.append(dev)
    return sorted(ports)


def parse_tray_status_line(line):
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
    def __init__(self, rx_queue, tx_queue):
        super().__init__(daemon=True)
        self.rx_queue = rx_queue
        self.tx_queue = tx_queue
        self.ser = None
        self._stop_evt = threading.Event()
        self._connected = threading.Event()
        self.status_poll_interval = 0.5
        self._last_poll = 0.0

    def connect(self, port, baud=9600):
        self.tx_queue.put(("__CONNECT__", {"port": port, "baud": baud}))

    def disconnect(self):
        self.tx_queue.put(("__DISCONNECT__", {}))

    def is_connected(self):
        return self._connected.is_set()

    def send(self, cmd):
        self.tx_queue.put(("CMD", {"cmd": cmd}))

    def stop(self):
        self._stop_evt.set()
        self.disconnect()

    def _do_connect(self, port, baud):
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            time.sleep(1.2)  # allow Arduino auto-reset
            self._connected.set()
            self.rx_queue.put(("INFO", "Tray connected: %s @ %d" % (port, baud)))
        except Exception as e:
            self._connected.clear()
            self.ser = None
            self.rx_queue.put(("ERR", "Tray connect failed: %s" % e))

    def _do_disconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._connected.clear()
        self.rx_queue.put(("INFO", "Tray disconnected"))

    def _write_line(self, s):
        if not self.ser or not self.is_connected():
            return
        try:
            self.ser.write((s.strip() + "\n").encode("utf-8"))
        except Exception as e:
            self.rx_queue.put(("ERR", "Tray write failed: %s" % e))
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
            self.rx_queue.put(("ERR", "Tray read failed: %s" % e))
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

    def __init__(self, port, baudrate=9600, slave_id=1):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.client = None
        self.connected = False
        self.lock = threading.Lock()

    def connect(self):
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

    def _read1(self, reg):
        if not self.connected or not self.client:
            return None
        try:
            rr = self.client.read_holding_registers(address=reg, count=1, slave=self.slave_id)
            if rr and not rr.isError():
                return rr.registers[0]
        except Exception:
            return None
        return None

    def _write1(self, reg, value):
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

    def set_output_enabled(self, enabled):
        return self._write1(self.REG_OUT_EN, 1 if enabled else 0)


# -------------------- PX100 driver adapter + controller --------------------
class SerialRawAdapter:
    """Adapter for PX100 driver transport expectations."""
    def __init__(self, port, baud=9600, timeout=2.0):
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def write_raw(self, b):
        return self._ser.write(b)

    def read_raw(self, n):
        return self._ser.read(n)

    def write_bytes(self, b):
        return self.write_raw(b)

    def read_bytes(self, n):
        return self.read_raw(n)

    @property
    def bytes_in_buffer(self):
        try:
            return int(self._ser.in_waiting)
        except Exception:
            return 0

    def flush(self):
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


def try_command(dev, cmd_name, value):
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


def normalize_readall(data):
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
    def __init__(self, port, baud=9600, timeout_s=2.0):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.transport = None
        self.dev = None
        self.connected = False
        self.lock = threading.Lock()
        self.tol_v = 0.01
        self.tol_a = 0.01

    def connect(self):
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

    def read_all(self):
        with self.lock:
            if not self.connected or not self.dev:
                return {}
            try:
                return normalize_readall(self.dev.readAll())
            except Exception:
                return {}

    def probe(self):
        return bool(self.read_all())

    def reset(self):
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_RESET, 0.0)  # your device expects 0.0
                time.sleep(0.5)
                return True
            except Exception:
                return False

    def set_load_enabled(self, enabled):
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_ENABLE, bool(enabled))
                return True
            except Exception:
                return False

    def _set_if_needed(self, cmd_name, current_val, desired_val, tol):
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

    def set_current_a(self, current_a):
        st = self.read_all()
        return self._set_if_needed(
            Instrument.COMMAND_SET_CURRENT,
            current_val=st.get("set_current"),
            desired_val=float(current_a),
            tol=self.tol_a,
        )

    def set_cutoff_v(self, cutoff_v):
        st = self.read_all()
        return self._set_if_needed(
            Instrument.COMMAND_SET_VOLTAGE,
            current_val=st.get("set_voltage"),
            desired_val=float(cutoff_v),
            tol=self.tol_v,
        )

    def start_capacity_test(self, current_a, cutoff_v):
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
        self.geometry("800x600")
        self.minsize(740, 520)

        # Tray
        self.tray_rx = queue.Queue()
        self.tray_tx = queue.Queue()
        self.tray = TraySerialWorker(self.tray_rx, self.tray_tx)
        self.tray.start()

        # PSU + PX100
        self.dps = None
        self.px100 = None

        # last PSU readings (for automation)
        self._psu_last_cur_a = None
        self._psu_last_out_en = None

        # Tray status snapshot (for automation)
        self._tray_status_latest = {}
        self._tray_status_ts = 0.0
        self._tray_empty_hint = False

        # Receipt printer
        self.receipt_printer_ip = "192.168.14.223"
        self.receipt_printer_port = 9100
        self.receipt_dots = 384  # good default for 54mm paper class printers

        # Image print "darkness" tuning
        self.receipt_img_contrast = 2.2
        self.receipt_img_sharpness = 1.5
        self.receipt_img_threshold = 180

        # PSU startup grace
        self.psu_on_grace_s = 4.0
        self._psu_on_cmd_ts = None

        # Contact timing / voltage gate
        self.contact_settle_s = 0.30       # after CONNECT before turning PSU on / enabling PX load
        self.precharge_settle_s = 2.0      # wait after CONNECT before reading voltage for charge gate
        self.min_charge_voltage = 2.0      # V
        self.auto_next_on_skip = True      # skip -> auto NEXT even if auto_run is off
        self._contact_cmd_ts = None

        # UI vars (ports only; baud is always 9600, PSU ID always 1)
        self.tray_port = tk.StringVar(value="")
        self.tray_conn_var = tk.StringVar(value="Tray: Disconnected")

        self.psu_port = tk.StringVar(value="")
        self.psu_conn_var = tk.StringVar(value="PSU: Disconnected")
        self.psu_current_var = tk.StringVar(value="--.- A")
        self.psu_out_var = tk.StringVar(value="--")

        self.px_port = tk.StringVar(value="")
        self.px_conn_var = tk.StringVar(value="PX100: Disconnected")

        # test settings
        self.px_set_current = tk.StringVar(value="2.00")
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

        # Automation checkboxes
        self.auto_charge = tk.BooleanVar(value=True)
        self.auto_test = tk.BooleanVar(value=True)
        self.auto_run = tk.BooleanVar(value=False)

        # charge cutoff behavior
        self.charge_cutoff_a = 0.10
        self.charge_stabilize_s = 10.0
        self._charge_low_since = None

        # process state
        self._proc_state = "IDLE"
        self._proc_cancel = False
        self.proc_state_var = tk.StringVar(value=self._proc_state)

        # per-cell timing
        self._cell_charge_start_ts = None
        self._cell_charge_end_ts = None
        self._cell_discharge_start_ts = None
        self._cell_discharge_end_ts = None

        # PX100 test logging
        self._test_running = False
        self._test_t0 = None
        self._test_samples = []   # list of {"t":..., "v":..., "mah":...}
        self._test_cutoff_v = None
        self._last_saved_path = None
        self._plot_last_n = 0
        self._live_fig = Figure(dpi=100, figsize=(4.0, 3.0))
        self._live_ax = self._live_fig.add_subplot(111)
        self._live_ax.set_title("Voltage vs Capacity")
        self._live_ax.set_xlabel("Capacity (mAh)")
        self._live_ax.set_ylabel("Voltage (V)")
        self._live_ax.grid(True, linewidth=0.5)
        (self._live_line,) = self._live_ax.plot([], [], linewidth=1.8)
        self._live_canvas = None

        self._build_ui()
        self._refresh_ports()

        self.after(50, self._pump_tray_rx)
        self.after(400, self._poll_psu)
        self.after(400, self._poll_px100)

    # ---------------- UI ----------------
    def _build_ui(self):
        style = ttk.Style(self)

        # ---- Compact styling for smaller screens ----
        base_btn = 12
        base_lbl = 10
        big_val  = 12  # bold readouts
        style.configure("TButton", font=("Sans", base_btn))
        style.configure("TLabel", font=("Sans", base_lbl))
        style.configure("Conn.TLabel", font=("Sans", 9))
        style.configure("Conn.TButton", font=("Sans", 9))
        style.configure("Value.TLabel", font=("Sans", big_val, "bold"))

        main = ttk.Frame(self, padding=8)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))

        right = ttk.Frame(main)
        right.pack(side="left", fill="both", expand=True)

        # ================= LEFT SIDE (TABS) =================
        left_nb = ttk.Notebook(left)
        left_nb.pack(fill="both", expand=True)

        tray_tab = ttk.Frame(left_nb, padding=6)
        auto_tab = ttk.Frame(left_nb, padding=6)
        psu_tab  = ttk.Frame(left_nb, padding=6)
        px_tab   = ttk.Frame(left_nb, padding=6)

        left_nb.add(tray_tab, text="Tray")
        left_nb.add(auto_tab, text="Auto")
        left_nb.add(psu_tab,  text="PSU")
        left_nb.add(px_tab,   text="PX100")

        # --- Tray controls ---
        tray_ctrl = ttk.LabelFrame(tray_tab, text="Tray Controls", padding=6)
        tray_ctrl.pack(fill="x")

        ttk.Button(tray_ctrl, text="LOAD", command=self._cmd_load).pack(fill="x", pady=2)
        ttk.Button(tray_ctrl, text="NEXT (cycle)", command=self._start_cycle_next_cell).pack(fill="x", pady=2)
        ttk.Button(tray_ctrl, text="START (current cell)", command=self._start_cycle_current_cell).pack(fill="x", pady=2)
        ttk.Button(tray_ctrl, text="UNLOAD", command=lambda: self._tray_send("UNLOAD")).pack(fill="x", pady=2)
        ttk.Separator(tray_ctrl).pack(fill="x", pady=6)
        ttk.Button(tray_ctrl, text="STOP", command=self._stop_cycle).pack(fill="x", pady=2)

        # --- Automation ---
        auto_ctrl = ttk.LabelFrame(auto_tab, text="Automation", padding=6)
        auto_ctrl.pack(fill="x")

        ttk.Checkbutton(auto_ctrl, text="Charge cell (gate: >=2.0V)", variable=self.auto_charge).pack(anchor="w")
        ttk.Checkbutton(auto_ctrl, text="Capacity test (PX100)", variable=self.auto_test).pack(anchor="w")
        ttk.Checkbutton(auto_ctrl, text="Auto-run all cells", variable=self.auto_run).pack(anchor="w", pady=(2, 0))

        ttk.Button(auto_ctrl, text="TEST PRINTER", command=self._printer_test).pack(fill="x", pady=(6, 0))

        # --- PSU ---
        psu_ctrl = ttk.LabelFrame(psu_tab, text="PSU Controls", padding=6)
        psu_ctrl.pack(fill="x")

        psu_btns = ttk.Frame(psu_ctrl)
        psu_btns.pack(fill="x")
        ttk.Button(psu_btns, text="PSU ON", command=self._psu_on).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(psu_btns, text="PSU OFF", command=self._psu_off).pack(side="left", expand=True, fill="x")

        # --- PX100 ---
        px_ctrl = ttk.LabelFrame(px_tab, text="PX100 Manual Test", padding=6)
        px_ctrl.pack(fill="x")

        row = ttk.Frame(px_ctrl)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Current (A):").pack(side="left")
        ttk.Entry(row, textvariable=self.px_set_current, width=8).pack(side="right")

        row2 = ttk.Frame(px_ctrl)
        row2.pack(fill="x", pady=2)
        ttk.Label(row2, text="Cutoff (V):").pack(side="left")
        ttk.Entry(row2, textvariable=self.px_set_cutoff, width=8).pack(side="right")

        px_btns = ttk.Frame(px_ctrl)
        px_btns.pack(fill="x", pady=(4, 0))
        ttk.Button(px_btns, text="START", command=self._px_start).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(px_btns, text="RESET", command=self._px_reset).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(px_btns, text="STOP", command=self._px_stop).pack(side="left", expand=True, fill="x")

        # ================= RIGHT SIDE (TABS) =================
        right_nb = ttk.Notebook(right)
        right_nb.pack(fill="both", expand=True)

        status_tab = ttk.Frame(right_nb, padding=6)
        log_tab    = ttk.Frame(right_nb, padding=6)
        conn_tab   = ttk.Frame(right_nb, padding=6)

        right_nb.add(status_tab, text="Status")
        right_nb.add(log_tab, text="Log")
        right_nb.add(conn_tab, text="Connections")

        # --- Status ---
        status = ttk.LabelFrame(status_tab, text="Overview", padding=6)
        status.pack(fill="x")

        grid = ttk.Frame(status)
        grid.pack(fill="both", expand=True)

        def srow(r, label, var):
            ttk.Label(grid, text=label).grid(row=r, column=0, sticky="w", padx=(0, 6), pady=1)
            ttk.Label(grid, textvariable=var, style="Value.TLabel").grid(row=r, column=1, sticky="w", pady=1)

        srow(0, "Cycle state:", self.proc_state_var)
        srow(1, "Tray detect:", self.tray_status_vars["detectClosed"])
        srow(2, "Tray home:", self.tray_status_vars["homeClosed"])
        srow(3, "Tray aligned:", self.tray_status_vars["aligned"])
        srow(4, "Tray contacts:", self.tray_status_vars["contacts"])
        srow(5, "Relay 1:", self.tray_status_vars["r1"])
        srow(6, "Relay 2:", self.tray_status_vars["r2"])

        ttk.Separator(status).pack(fill="x", pady=4)

        srow(7, "PSU output:", self.psu_out_var)
        srow(8, "PSU current:", self.psu_current_var)

        ttk.Separator(status).pack(fill="x", pady=4)

        srow(9, "PX100 voltage:", self.px_v_var)
        srow(10, "PX100 current:", self.px_i_var)
        srow(11, "PX100 cap:", self.px_mah_var)
        srow(12, "PX100 time:", self.px_time_var)
        srow(13, "PX100 temp:", self.px_temp_var)

        live = ttk.LabelFrame(status_tab, text="Capacity (live)", padding=6)
        live.pack(fill="both", expand=True, pady=(6, 0))
        self._live_canvas = FigureCanvasTkAgg(self._live_fig, master=live)
        self._live_canvas.draw()
        self._live_canvas.get_tk_widget().pack(fill="both", expand=True)

        # --- Log ---
        logf = ttk.LabelFrame(log_tab, text="Log", padding=6)
        logf.pack(fill="both", expand=True)

        self.log = tk.Text(logf, height=8, wrap="word", font=("Sans", 10))
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")

        # ================= CONNECTIONS TAB =================
        conn_box = ttk.LabelFrame(conn_tab, text="Serial Connections", padding=6)
        conn_box.pack(fill="both", expand=True)

        row = ttk.Frame(conn_box)
        row.pack(fill="x")

        ttk.Button(row, text="Refresh Ports", command=self._refresh_ports, style="Conn.TButton").pack(side="left", padx=(0, 6))
        ttk.Button(row, text="Auto Detect", command=self._auto_detect_ports, style="Conn.TButton").pack(side="left")

        ttk.Label(row, textvariable=self.tray_conn_var, style="Conn.TLabel").pack(side="right", padx=(8, 0))
        ttk.Label(row, textvariable=self.psu_conn_var, style="Conn.TLabel").pack(side="right", padx=(8, 0))
        ttk.Label(row, textvariable=self.px_conn_var, style="Conn.TLabel").pack(side="right", padx=(8, 0))

        ttk.Separator(conn_box).pack(fill="x", pady=6)

        def conn_row(label, combo_var, on_conn, on_disc):
            r = ttk.Frame(conn_box)
            r.pack(fill="x", pady=3)
            ttk.Label(r, text=label, style="Conn.TLabel").pack(side="left")
            combo = ttk.Combobox(r, textvariable=combo_var, width=16, state="readonly")
            combo.pack(side="left", padx=6)
            ttk.Button(r, text="Connect", command=on_conn, style="Conn.TButton").pack(side="left")
            ttk.Button(r, text="Disconnect", command=on_disc, style="Conn.TButton").pack(side="left", padx=(6, 0))
            return combo

        self.tray_combo = conn_row("Tray", self.tray_port, self._tray_connect, self._tray_disconnect)
        self.psu_combo = conn_row("PSU", self.psu_port, self._psu_connect, self._psu_disconnect)
        self.px_combo = conn_row("PX100", self.px_port, self._px_connect, self._px_disconnect)

    # ---------------- common helpers ----------------
    def _log(self, msg):
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

    def _auto_detect_ports(self):
        found = auto_detect_ports()
        if found.get("tray"):
            self.tray_port.set(found["tray"])
        if found.get("psu"):
            self.psu_port.set(found["psu"])
        if found.get("px100"):
            self.px_port.set(found["px100"])

        msg = "Auto-detect: tray=%s psu=%s px100=%s" % (
            found.get("tray") or "--",
            found.get("psu") or "--",
            found.get("px100") or "--",
        )
        self._log(msg)

    def _fmt_duration(self, seconds):
        if seconds is None:
            return "--:--"
        try:
            s = int(round(float(seconds)))
        except Exception:
            return "--:--"
        m = s // 60
        r = s % 60
        if m < 100:
            return "%02d:%02d" % (m, r)
        h = m // 60
        m2 = m % 60
        return "%d:%02d:%02d" % (h, m2, r)

    def _tray_status_fresh(self, max_age_s=2.0):
        return (time.time() - self._tray_status_ts) <= max_age_s

    def _tray_cell_looks_loaded(self):
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

    # ---- Contacts management ----
    def _contacts_connect(self):
        if self.tray and self.tray.is_connected():
            self._tray_send("CONNECT")
            self._contact_cmd_ts = time.time()

    def _contacts_disconnect(self):
        if self.tray and self.tray.is_connected():
            self._tray_send("DISCONNECT")
            self._contact_cmd_ts = time.time()

    def _disconnect_from_cell(self):
        """Ensure PSU/PX load OFF + tray contacts OPEN (prevents parasitic draw)."""
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
        try:
            self._contacts_disconnect()
        except Exception:
            pass

    def _abort_with_disconnect(self, msg):
        self._log(msg)
        self._disconnect_from_cell()
        self._proc_state = "IDLE"
        self.proc_state_var.set(self._proc_state)

    # ---------------- Tray actions ----------------
    def _tray_connect(self):
        port = self.tray_port.get().strip()
        if not port:
            messagebox.showwarning("Tray", "Select a tray serial port.")
            return
        self.tray.connect(port, 9600)

    def _tray_disconnect(self):
        self.tray.disconnect()

    def _tray_send(self, cmd):
        self.tray.send(cmd)

    def _cmd_load(self):
        # LOAD = HOME then RECONNECT (Arduino HOME leaves contacts OPEN)
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

        baud = 9600
        slave = 1
        self.dps = DPS5020(port, baudrate=baud, slave_id=slave)
        if self.dps.connect():
            self.psu_conn_var.set("PSU: Connected %s @ %d, ID %d" % (port, baud, slave))
            self._log("PSU connected: %s (baud %d, id %d)" % (port, baud, slave))
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
        self._psu_on_cmd_ts = time.time()
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
            self.psu_current_var.set("--.- A" if cur_a is None else "%.2f A" % cur_a)
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

        self.px100 = PX100Controller(port, baud=9600, timeout_s=2.0)
        if self.px100.connect():
            self.px_conn_var.set("PX100: Connected %s @ 9600" % port)
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
        # Safety: open contacts after manual stop
        self._contacts_disconnect()

    def _px_start(self):
        """Manual start (also used by automation). Returns True/False."""
        if not self.px100 or not self.px100.connected:
            self._log("ERROR: PX100 not connected")
            return False

        # ALWAYS force PSU OFF before enabling PX100 load
        try:
            if self.dps and self.dps.connected:
                self.dps.set_output_enabled(False)
                self._log("PX100: ensuring PSU is OFF before test")
        except Exception as e:
            self._log("WARNING: could not force PSU OFF: %s" % e)

        try:
            current_a = float(self.px_set_current.get().strip())
            cutoff_v = float(self.px_set_cutoff.get().strip())
        except ValueError:
            messagebox.showwarning("PX100", "Enter numeric current and cutoff voltage.")
            return False

        # Reset before test
        self._log("PX100: Resetting before test…")
        try:
            self.px100.reset()
        except Exception:
            pass
        time.sleep(0.2)

        # Start logging
        self._test_samples = []
        self._plot_last_n = 0
        if self._live_canvas:
            self._live_line.set_data([], [])
            self._live_ax.relim()
            self._live_ax.autoscale_view()
            self._live_canvas.draw_idle()
        self._test_t0 = time.time()
        self._test_running = True
        self._test_cutoff_v = cutoff_v

        # discharge timer
        self._cell_discharge_start_ts = time.time()
        self._cell_discharge_end_ts = None

        ok = self.px100.start_capacity_test(current_a=current_a, cutoff_v=cutoff_v)
        self._log("PX100 START TEST" if ok else "ERROR: PX100 start failed")
        if not ok:
            self._test_running = False
        return ok

    def _px_extract_mah(self, row):
        cap_mah = safe_float(row.get("cap_mah"))
        if cap_mah is not None:
            return cap_mah
        cap_ah = safe_float(row.get("cap_ah"))
        if cap_ah is not None:
            return cap_ah * 1000.0
        for k in ("capacity_mah", "mah"):
            v = safe_float(row.get(k))
            if v is not None:
                return v
        for k in ("capacity_ah", "ah"):
            v = safe_float(row.get(k))
            if v is not None:
                return v * 1000.0
        return None

    def _px_record_sample(self, row):
        if not self._test_running or self._test_t0 is None:
            return
        v = safe_float(row.get("voltage"))
        mah = self._px_extract_mah(row)
        if v is None and mah is None:
            return
        t = time.time() - self._test_t0
        self._test_samples.append({"t": t, "v": v, "mah": mah})

    def _update_live_plot(self):
        if not self._live_canvas:
            return
        n = len(self._test_samples)
        if n == 0:
            if self._plot_last_n != 0:
                self._live_line.set_data([], [])
                self._live_ax.relim()
                self._live_ax.autoscale_view()
                self._live_canvas.draw_idle()
                self._plot_last_n = 0
            return
        if n == self._plot_last_n:
            return

        xs = [s["mah"] for s in self._test_samples if s.get("mah") is not None and s.get("v") is not None]
        ys = [s["v"] for s in self._test_samples if s.get("mah") is not None and s.get("v") is not None]
        self._plot_last_n = n
        if not xs or not ys:
            return

        self._live_line.set_data(xs, ys)
        self._live_ax.relim()
        self._live_ax.autoscale_view()
        self._live_canvas.draw_idle()

    def _poll_px100(self):
        if self.px100 and self.px100.connected:
            row = self.px100.read_all()

            v = safe_float(row.get("voltage"))
            i = safe_float(row.get("current"))
            temp = safe_float(row.get("temp"))
            mah = self._px_extract_mah(row)

            self.px_v_var.set("--.- V" if v is None else "%.3f V" % v)
            self.px_i_var.set("--.- A" if i is None else "%.3f A" % i)
            self.px_temp_var.set("-- °C" if temp is None else "%.1f °C" % temp)
            self.px_mah_var.set("---- mAh" if mah is None else "%.0f mAh" % mah)

            tval = row.get("time")
            tstr = None
            if isinstance(tval, (int, float)):
                sec = int(max(0, tval))
                h = sec // 3600
                m = (sec % 3600) // 60
                s = sec % 60
                tstr = "%02d:%02d:%02d" % (h, m, s)
            elif isinstance(tval, str):
                tstr = tval
            elif tval is not None:
                tstr = str(tval)
            self.px_time_var.set("--:--:--" if not tstr else tstr)

            # record samples
            if self._test_running:
                self._px_record_sample(row)
                self._update_live_plot()

                # stop condition by cutoff voltage
                if self._test_cutoff_v is not None and v is not None and v <= float(self._test_cutoff_v):
                    self._log("PX100: Cutoff reached (V=%.3f <= %.3f). Stopping test." % (v, float(self._test_cutoff_v)))
                    try:
                        self.px100.set_load_enabled(False)
                    except Exception:
                        pass
                    self._test_running = False

                    # mark discharge end
                    self._cell_discharge_end_ts = time.time()

                    self._on_capacity_test_finished()

        self.after(400, self._poll_px100)

    # ---------------- Graph image prep for darker receipt printing ----------------
    def _prepare_image_for_receipt(self, path):
        from PIL import ImageEnhance
        with Image.open(path) as im:
            im = im.convert("L")
            im = ImageEnhance.Contrast(im).enhance(float(self.receipt_img_contrast))
            im = ImageEnhance.Sharpness(im).enhance(float(self.receipt_img_sharpness))

            thr = int(self.receipt_img_threshold)
            im = im.point(lambda p: 255 if p > thr else 0, mode="1")
            im.save(path)

    # ---------------- Graph save (voltage only) ----------------
    def _save_graph_png_auto(self, folder, capacity_mah):
        os.makedirs(folder, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        cap_str = "unknown"
        if capacity_mah is not None:
            try:
                cap_str = "%.0fmAh" % float(capacity_mah)
            except Exception:
                cap_str = "unknown"

        filename = "capacity_%s_%s.png" % (cap_str, ts)
        path = os.path.join(folder, filename)

        # Build 800x600
        width_px, height_px = 800, 600
        dpi = 100
        fig = Figure(dpi=dpi, figsize=(width_px / dpi, height_px / dpi))
        ax = fig.add_subplot(111)

        x_v = [s["t"] for s in self._test_samples if s.get("t") is not None and s.get("v") is not None]
        y_v = [s["v"] for s in self._test_samples if s.get("t") is not None and s.get("v") is not None]
        if x_v:
            ax.plot(x_v, y_v, linewidth=2.0)

        ax.set_xlabel("")
        ax.set_ylabel("")
        ax.set_title("")
        ax.tick_params(labelsize=8)
        ax.grid(True, linewidth=0.5)

        fig.tight_layout()
        fig.savefig(path)

        # Shrink to receipt width (keep aspect)
        try:
            with Image.open(path) as im:
                if im.mode not in ("RGB", "L"):
                    im = im.convert("RGB")
                target_w = int(getattr(self, "receipt_dots", 384))
                im.thumbnail((target_w, 10000), Image.LANCZOS)
                im.save(path)
        except Exception as e:
            self._log("WARNING: could not thumbnail graph for receipt: %s" % e)
            return path

        # Darken/threshold for printer
        try:
            self._prepare_image_for_receipt(path)
        except Exception as e:
            self._log("WARNING: could not prepare image for receipt: %s" % e)

        return path

    # ---------------- Receipt printing ----------------
    def _print_receipt_with_graph(self, graph_png_path, capacity_mah, charge_s, discharge_s, test_current_a, cutoff_v):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            p = Network(self.receipt_printer_ip, port=self.receipt_printer_port, timeout=10)

            p.set(align="center", bold=True, width=2, height=2)
            p.text("CELL TEST\n")

            p.set(align="center", bold=False, width=1, height=1)
            p.text(ts + "\n\n")

            if capacity_mah is None:
                p.text("CAP: ---- mAh\n")
            else:
                p.set(bold=True)
                p.text("CAP: %.0f mAh\n" % float(capacity_mah))
                p.set(bold=False)

            if test_current_a is None:
                test_current_a = safe_float(self.px_set_current.get())
            if cutoff_v is None:
                cutoff_v = safe_float(self.px_set_cutoff.get())

            if test_current_a is None:
                p.text("I: --.-A  Vcut: --.--V\n")
            else:
                p.text("I: %.2fA  Vcut: %.2fV\n" % (float(test_current_a), float(cutoff_v) if cutoff_v is not None else 0.0))

            p.text("Tchg:%s  Tdis:%s\n" % (self._fmt_duration(charge_s), self._fmt_duration(discharge_s)))
            p.text("\n")
            p.cut()
        except Exception as e:
            self._log("CYCLE: ERROR printing receipt: %s" % e)

    def _print_skip_receipt(self, reason, voltage=None):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            p = Network(self.receipt_printer_ip, port=self.receipt_printer_port, timeout=10)

            p.set(align="center", bold=True, width=2, height=2)
            p.text("CELL SKIPPED\n")

            p.set(align="center", bold=False, width=1, height=1)
            p.text(ts + "\n\n")

            p.set(align="left", bold=True)
            p.text("Reason:\n")
            p.set(bold=False)
            p.text(str(reason).strip() + "\n\n")

            if voltage is not None:
                try:
                    p.text("V: %.3f V\n" % float(voltage))
                except Exception:
                    p.text("V: %s\n" % str(voltage))

            p.text("\n")
            p.cut()
        except Exception as e:
            self._log("WARNING: could not print SKIPPED receipt: %s" % e)

    # ---------------- Printer test (dummy finished receipt) ----------------
    def _make_dummy_graph_png(self, out_path):
        total_s = 45 * 60
        step_s = 5
        n = total_s // step_s + 1

        cutoff_v = safe_float(self.px_set_cutoff.get()) or 3.00
        start_v = 4.15
        end_v = cutoff_v

        xs, vs = [], []
        cap_final_mah = 3100.0

        for i in range(int(n)):
            t = i * step_s
            frac = t / float(total_s)

            if frac < 0.08:
                v = start_v - 0.25 * (frac / 0.08)
            elif frac < 0.85:
                v = (start_v - 0.25) - 0.35 * ((frac - 0.08) / 0.77)
            else:
                v = (start_v - 0.25 - 0.35) - 0.55 * ((frac - 0.85) / 0.15)

            v = max(end_v, min(start_v, v))
            xs.append(t)
            vs.append(v)

        width_px, height_px = 800, 600
        dpi = 100
        fig = Figure(dpi=dpi, figsize=(width_px / dpi, height_px / dpi))
        ax = fig.add_subplot(111)
        ax.plot(xs, vs, linewidth=2.0)
        ax.set_xlabel("")
        ax.set_ylabel("")
        ax.set_title("")
        ax.tick_params(labelsize=8)
        ax.grid(True, linewidth=0.5)
        fig.tight_layout()
        fig.savefig(out_path)

        with Image.open(out_path) as im:
            if im.mode not in ("RGB", "L"):
                im = im.convert("RGB")
            target_w = int(getattr(self, "receipt_dots", 384))
            im.thumbnail((target_w, 10000), Image.LANCZOS)
            im.save(out_path)

        self._prepare_image_for_receipt(out_path)
        return cap_final_mah

    def _printer_test(self):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        folder = os.path.join(os.getcwd(), "capacity_graphs")
        os.makedirs(folder, exist_ok=True)
        tmp_path = os.path.join(folder, "printer_test_dummy_graph.png")

        try:
            cap_final_mah = self._make_dummy_graph_png(tmp_path)
        except Exception as e:
            messagebox.showerror("Printer test", "Failed to generate dummy graph:\n%s" % e)
            self._log("ERROR: dummy graph generation failed: %s" % e)
            return

        try:
            current_a = safe_float(self.px_set_current.get()) or 1.00
            cutoff_v = safe_float(self.px_set_cutoff.get()) or 3.00

            p = Network(self.receipt_printer_ip, port=self.receipt_printer_port, timeout=10)

            p.set(align="center", bold=True, width=2, height=2)
            p.text("TEST RECEIPT\n")

            p.set(align="center", bold=False, width=1, height=1)
            p.text(ts + "\n\n")

            p.set(bold=True)
            p.text("CAP: %.0f mAh\n" % cap_final_mah)
            p.set(bold=False)

            p.text("I: %.2fA  Vcut: %.2fV\n" % (current_a, cutoff_v))
            p.text("Tchg:%s  Tdis:%s\n" % ("12:34", "45:00"))
            p.text("\n")

            p.set(align="center")
            p.image(tmp_path)

            p.text("\n")
            p.cut()

            self._log("Printer test sent (dummy graph) -> %s:%d" % (self.receipt_printer_ip, self.receipt_printer_port))
        except Exception as e:
            messagebox.showerror("Printer test", "Failed to print:\n%s" % e)
            self._log("ERROR: printer test failed: %s" % e)

    # ---------------- Automation cycle ----------------
    def _stop_cycle(self):
        self._proc_cancel = True
        self._proc_state = "IDLE"
        self.proc_state_var.set(self._proc_state)
        self._charge_low_since = None
        self._test_running = False
        self._psu_on_cmd_ts = None
        self._log("CYCLE: cancelled")

        # safety: turn off PSU + PX100 load + open contacts
        self._disconnect_from_cell()

    def _start_cycle_next_cell(self):
        if self._proc_state != "IDLE":
            self._log("CYCLE: already running")
            return

        if not self.tray.is_connected():
            self._abort_with_disconnect("ERROR: Tray not connected")
            return

        if self.auto_charge.get() and (not self.dps or not self.dps.connected):
            self._abort_with_disconnect("ERROR: Charge enabled but PSU not connected")
            return

        if (self.auto_charge.get() or self.auto_test.get()) and (not self.px100 or not self.px100.connected):
            # We use PX100 for voltage gate + discharge
            self._abort_with_disconnect("ERROR: PX100 required but not connected")
            return

        # reset per-cell timings
        self._cell_charge_start_ts = None
        self._cell_charge_end_ts = None
        self._cell_discharge_start_ts = None
        self._cell_discharge_end_ts = None

        self._proc_cancel = False
        self._tray_empty_hint = False

        # ensure disconnected before moving
        self._contacts_disconnect()

        # clear stale tray status so NEXT doesn't reuse previous cell status
        self._tray_status_latest = {}
        self._tray_status_ts = 0.0

        self._proc_state = "MOVE_NEXT"
        self.proc_state_var.set(self._proc_state)
        self._log("CYCLE: NEXT cell…")
        self._tray_send("NEXT")  # Arduino NEXT ends with RECONNECT
        self.after(350, self._cycle_tick)

    def _start_cycle_current_cell(self):
        if self._proc_state != "IDLE":
            self._log("CYCLE: already running")
            return

        if not self.tray.is_connected():
            self._abort_with_disconnect("ERROR: Tray not connected")
            return

        if self.auto_charge.get() and (not self.dps or not self.dps.connected):
            self._abort_with_disconnect("ERROR: Charge enabled but PSU not connected")
            return

        if (self.auto_charge.get() or self.auto_test.get()) and (not self.px100 or not self.px100.connected):
            # We use PX100 for voltage gate + discharge
            self._abort_with_disconnect("ERROR: PX100 required but not connected")
            return

        # reset per-cell timings
        self._cell_charge_start_ts = None
        self._cell_charge_end_ts = None
        self._cell_discharge_start_ts = None
        self._cell_discharge_end_ts = None

        self._proc_cancel = False
        self._tray_empty_hint = False

        # ensure disconnected before starting
        self._contacts_disconnect()

        self._proc_state = "WAIT_CELL"
        self.proc_state_var.set(self._proc_state)
        self._log("CYCLE: start current cell…")
        self._wait_cell_start = time.time()
        self.after(200, self._cycle_tick)

    def _skip_and_advance(self, reason, voltage=None):
        self._log(f"CYCLE: SKIP -> {reason}")
        self._disconnect_from_cell()
        self._print_skip_receipt(reason=reason, voltage=voltage)

        do_next = self.auto_run.get() or self.auto_next_on_skip
        if do_next and self.tray and self.tray.is_connected():
            self._log("CYCLE: skip -> auto NEXT…")
            self._proc_state = "MOVE_NEXT"
            self.proc_state_var.set(self._proc_state)
            self._tray_empty_hint = False

            # reset per-cell timings
            self._cell_charge_start_ts = None
            self._cell_charge_end_ts = None
            self._cell_discharge_start_ts = None
            self._cell_discharge_end_ts = None

            self._tray_send("NEXT")
            self.after(350, self._cycle_tick)
            return

        self._proc_state = "IDLE"
        self.proc_state_var.set(self._proc_state)

    def _cycle_tick(self):
        self.proc_state_var.set(self._proc_state)
        if self._proc_cancel:
            self._proc_state = "IDLE"
            self.proc_state_var.set(self._proc_state)
            return

        if self._tray_empty_hint:
            self._log("CYCLE: tray reported empty / no more cells. Stopping.")
            self._disconnect_from_cell()
            self._proc_state = "IDLE"
            self.proc_state_var.set(self._proc_state)
            return

        if self._proc_state == "MOVE_NEXT":
            # After NEXT, immediately open contacts to avoid parasitic draw while waiting
            self._contacts_disconnect()
            self._proc_state = "WAIT_CELL"
            self._wait_cell_start = time.time()
            self.after(200, self._cycle_tick)
            return

        if self._proc_state == "WAIT_CELL":
            if self._tray_cell_looks_loaded():
                self._log("CYCLE: cell loaded. Opening contacts (idle-safe)…")
                self._contacts_disconnect()

                if self.auto_charge.get():
                    # We'll connect just for voltage check, then disconnect again
                    self._proc_state = "VCHK_CONNECT"
                elif self.auto_test.get():
                    self._proc_state = "TEST_CONNECT"
                else:
                    self._proc_state = "FINISH"

                self.after(150, self._cycle_tick)
                return

            if (time.time() - self._wait_cell_start) > 6.0:
                self._abort_with_disconnect("CYCLE: could not confirm a cell (empty tray or jam). Stopping.")
                return

            self.after(200, self._cycle_tick)
            return

        # ---- Voltage check for charge gate: CONNECT -> wait -> read -> DISCONNECT ----
        if self._proc_state == "VCHK_CONNECT":
            self._log("CYCLE: connecting contacts for voltage check…")
            self._contacts_connect()
            self._vchk_start = time.time()
            self._proc_state = "VCHK_WAIT"
            self.after(100, self._cycle_tick)
            return

        if self._proc_state == "VCHK_WAIT":
            if (time.time() - self._vchk_start) < self.precharge_settle_s:
                self.after(200, self._cycle_tick)
                return

            v = None
            try:
                if self.px100 and self.px100.connected:
                    row = self.px100.read_all()
                    v = safe_float(row.get("voltage"))
            except Exception:
                v = None

            # Always disconnect after voltage check
            self._contacts_disconnect()

            if v is None:
                self._skip_and_advance("Voltage read failed", voltage=None)
                return

            if v < self.min_charge_voltage:
                self._skip_and_advance(f"Low voltage ({v:.2f}V < {self.min_charge_voltage:.2f}V)", voltage=v)
                return

            self._log(f"CYCLE: voltage OK ({v:.2f} V). Preparing to charge…")
            self._proc_state = "CHARGE_CONNECT"
            self.after(150, self._cycle_tick)
            return

        # ---- Charge: CONNECT -> settle -> PSU ON -> wait cutoff -> PSU OFF -> DISCONNECT ----
        if self._proc_state == "CHARGE_CONNECT":
            self._log("CYCLE: connecting contacts for charge…")
            self._contacts_connect()
            self._proc_state = "CHARGE_START"
            self.after(int(self.contact_settle_s * 1000), self._cycle_tick)
            return

        if self._proc_state == "CHARGE_START":
            self._log("CYCLE: charging (PSU ON)…")
            self._charge_low_since = None
            self._psu_on_cmd_ts = time.time()

            self._cell_charge_start_ts = time.time()
            self._cell_charge_end_ts = None

            ok = self.dps.set_output_enabled(True)
            if not ok:
                self._abort_with_disconnect("CYCLE: ERROR turning PSU ON. Stopping.")
                return

            self._proc_state = "CHARGE_WAIT"
            self.after(400, self._cycle_tick)
            return

        if self._proc_state == "CHARGE_WAIT":
            cur = self._psu_last_cur_a
            out_en = self._psu_last_out_en
            now = time.time()

            if self._psu_on_cmd_ts is not None and (now - self._psu_on_cmd_ts) < self.psu_on_grace_s:
                self.after(400, self._cycle_tick)
                return

            if out_en == 0:
                self._abort_with_disconnect("CYCLE: PSU output is OFF unexpectedly (after grace). Stopping.")
                return

            if cur is None:
                self.after(400, self._cycle_tick)
                return

            if cur <= self.charge_cutoff_a:
                if self._charge_low_since is None:
                    self._charge_low_since = now
                    self._log("CYCLE: current low, waiting 10s stabilize…")
                low_for = now - self._charge_low_since
                if low_for >= self.charge_stabilize_s:
                    self._log("CYCLE: charge complete (cutoff stable). PSU OFF.")
                    self.dps.set_output_enabled(False)

                    self._cell_charge_end_ts = time.time()
                    self._charge_low_since = None
                    self._psu_on_cmd_ts = None

                    self._proc_state = "CHARGE_DISCONNECT"
                    self._log("CYCLE: opening contacts after charge…")
                    self._contacts_disconnect()
                    self.after(200, self._cycle_tick)
                    return
            else:
                self._charge_low_since = None

            self.after(400, self._cycle_tick)
            return

        if self._proc_state == "CHARGE_DISCONNECT":
            # proceed to test or finish; contacts already open
            self._proc_state = "TEST_CONNECT" if self.auto_test.get() else "FINISH"
            self.after(100, self._cycle_tick)
            return

        # ---- Discharge: CONNECT -> settle -> start test -> wait -> DISCONNECT ----
        if self._proc_state == "TEST_CONNECT":
            # ensure PSU off before discharge
            try:
                if self.dps and self.dps.connected:
                    self.dps.set_output_enabled(False)
            except Exception:
                pass

            self._log("CYCLE: connecting contacts for discharge…")
            self._contacts_connect()
            self._proc_state = "TEST_START"
            self.after(int(self.contact_settle_s * 1000), self._cycle_tick)
            return

        if self._proc_state == "TEST_START":
            self._log("CYCLE: starting capacity test…")
            ok = self._px_start()
            if not ok:
                self._abort_with_disconnect("CYCLE: ERROR starting PX100 test. Stopping.")
                return
            self._proc_state = "TEST_WAIT"
            self.after(500, self._cycle_tick)
            return

        if self._proc_state == "TEST_WAIT":
            if self._test_running:
                self.after(500, self._cycle_tick)
                return
            # test finished -> ensure contacts are open
            self._proc_state = "FINISH"
            self.after(100, self._cycle_tick)
            return

        if self._proc_state == "FINISH":
            self._log("CYCLE: finished cell. Opening contacts…")
            self._disconnect_from_cell()

            if self.auto_run.get():
                self._log("CYCLE: auto-run -> next cell…")
                self._proc_state = "MOVE_NEXT"
                self.proc_state_var.set(self._proc_state)
                self._tray_empty_hint = False

                # clear stale tray status so NEXT doesn't reuse previous cell status
                self._tray_status_latest = {}
                self._tray_status_ts = 0.0

                # reset per-cell timings
                self._cell_charge_start_ts = None
                self._cell_charge_end_ts = None
                self._cell_discharge_start_ts = None
                self._cell_discharge_end_ts = None

                self._tray_send("NEXT")
                self.after(350, self._cycle_tick)
                return

            self._proc_state = "IDLE"
            self.proc_state_var.set(self._proc_state)
            return

    def _on_capacity_test_finished(self):
        # final capacity from last sample with mah
        cap = None
        for s in reversed(self._test_samples):
            cap = safe_float(s.get("mah"))
            if cap is not None:
                break

        charge_s = None
        if self._cell_charge_start_ts is not None and self._cell_charge_end_ts is not None:
            charge_s = self._cell_charge_end_ts - self._cell_charge_start_ts

        discharge_s = None
        if self._cell_discharge_start_ts is not None and self._cell_discharge_end_ts is not None:
            discharge_s = self._cell_discharge_end_ts - self._cell_discharge_start_ts

        test_current_a = safe_float(self.px_set_current.get())
        cutoff_v = safe_float(self.px_set_cutoff.get())

        folder = os.path.join(os.getcwd(), "capacity_graphs")
        path = None
        try:
            path = self._save_graph_png_auto(folder=folder, capacity_mah=cap)
            self._last_saved_path = path
            if path:
                self._log("CYCLE: saved graph -> %s" % path)
        except Exception as e:
            self._log("CYCLE: ERROR saving graph: %s" % e)

        if path:
            self._log("CYCLE: printing receipt…")
            self._print_receipt_with_graph(path, cap, charge_s, discharge_s, test_current_a, cutoff_v)

        # IMPORTANT: open contacts after discharge to prevent parasitic drain
        self._log("CYCLE: discharge finished. Opening contacts…")
        self._contacts_disconnect()

        # advance state machine if waiting
        if self._proc_state == "TEST_WAIT":
            self._proc_state = "FINISH"
            self.after(100, self._cycle_tick)


if __name__ == "__main__":
    app = App()
    app.mainloop()
