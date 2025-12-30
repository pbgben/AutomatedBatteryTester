#!/usr/bin/env python3
import os
import sys

from devices.ports import auto_detect_ports
from devices.tray import TrayDevice
from devices.psu_dps5020 import DPS5020
from devices.px100_ctrl import PX100Controller

from reporting.receipt import ReceiptPrinter
from reporting.graph import GraphMaker

from automation.runner import AutomationRunner
from ui.app import App


def main():
    # Auto-detect ports from VID/PID
    found = auto_detect_ports()

    tray = TrayDevice()
    psu = None
    px = None

    # Printer + graph tools
    printer = ReceiptPrinter(ip="192.168.14.223", port=9100, receipt_dots=384)
    graphs = GraphMaker(receipt_dots=384)

    # Automation runner
    runner = AutomationRunner(
        tray=tray,
        psu_factory=lambda port: DPS5020(port, baudrate=9600, slave_id=1),
        px_factory=lambda port: PX100Controller(port, baud=9600, timeout_s=2.0),
        printer=printer,
        graphs=graphs,
        output_dir=os.path.join(os.getcwd(), "capacity_graphs"),
    )

    app = App(
        tray=tray,
        runner=runner,
        ports_detected=found,
    )

    app.mainloop()

    # Cleanup
    try:
        runner.stop()
    except Exception:
        pass
    try:
        tray.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
