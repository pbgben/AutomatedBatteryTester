import threading
import queue
import time
import serial

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


class TrayDevice:
    """
    Owns the serial port and a background reader thread.
    Provides send(cmd), connect(port), disconnect().
    Maintains latest STATUS dict.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._ser = None
        self._rx_thread = None
        self._stop_evt = threading.Event()

        self.connected = False
        self.port = None
        self.baud = 9600

        self.rx_lines = queue.Queue()
        self.status = {}
        self.status_ts = 0.0

        self._last_poll = 0.0
        self.status_poll_interval = 0.5

    def connect(self, port, baud=9600):
        with self._lock:
            self.disconnect()
            self.port = port
            self.baud = baud
            self._stop_evt.clear()
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            time.sleep(1.2)  # Arduino auto-reset
            self.connected = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            return True

    def disconnect(self):
        with self._lock:
            self.connected = False
            self._stop_evt.set()
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None

    def close(self):
        self.disconnect()

    def is_connected(self):
        return bool(self.connected and self._ser)

    def send(self, cmd):
        with self._lock:
            if not self._ser or not self.connected:
                return False
            try:
                self._ser.write((cmd.strip() + "\n").encode("utf-8"))
                return True
            except Exception:
                self.connected = False
                return False

    def _rx_loop(self):
        while not self._stop_evt.is_set():
            # periodic STATUS poll
            now = time.time()
            if self.connected and self._ser and (now - self._last_poll) >= self.status_poll_interval:
                self._last_poll = now
                try:
                    self._ser.write(b"STATUS\n")
                except Exception:
                    self.connected = False

            # read lines
            if not self.connected or not self._ser:
                time.sleep(0.1)
                continue

            try:
                raw = self._ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.rx_lines.put(line)
                        st = parse_tray_status_line(line)
                        if st:
                            self.status = st
                            self.status_ts = time.time()
            except Exception:
                self.connected = False

            time.sleep(0.01)
