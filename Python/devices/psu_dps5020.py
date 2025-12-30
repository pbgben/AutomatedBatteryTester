import threading
from pymodbus.client import ModbusSerialClient

class DPS5020:
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
