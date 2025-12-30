import time
import threading
import serial

# These must exist in your repo (as you showed earlier)
from instruments.instrument import Instrument
from instruments.px100 import PX100 as PX100_Driver


def safe_float(x):
    try:
        return float(x)
    except Exception:
        return None


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


def try_command(dev, cmd_name, value):
    try:
        return dev.command(cmd_name, value)
    except TypeError:
        pass
    try:
        return dev.command({cmd_name: value})
    except TypeError:
        pass
    raise


class SerialRawAdapter:
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


class PX100Controller:
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

    def read_voltage_v(self):
        row = self.read_all()
        return safe_float(row.get("voltage"))

    def set_load_enabled(self, enabled):
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_ENABLE, bool(enabled))
                return True
            except Exception:
                return False

    def reset(self):
        with self.lock:
            if not self.connected or not self.dev:
                return False
            try:
                try_command(self.dev, Instrument.COMMAND_RESET, 0.0)
                time.sleep(0.5)
                return True
            except Exception:
                return False

    def _set_if_needed(self, cmd_name, current_val, desired_val, tol):
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
        with self.lock:
            if not self.connected or not self.dev:
                return False
            return self._set_if_needed(
                Instrument.COMMAND_SET_CURRENT,
                current_val=st.get("set_current"),
                desired_val=float(current_a),
                tol=self.tol_a,
            )

    def set_cutoff_v(self, cutoff_v):
        st = self.read_all()
        with self.lock:
            if not self.connected or not self.dev:
                return False
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
