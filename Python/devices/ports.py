import serial.tools.list_ports

def auto_detect_ports():
    """
    Uses your observed list_ports values:
      PX100: VID:PID = 1027:24577 (0x0403:0x6001)
      PSU:   VID:PID = 6790:29987 (0x1a86:0x7523)
      Tray:  VID:PID = 9025:66    (0x2341:0x0042)
    Returns dict: {"tray": "/dev/ttyACM0", "psu": "/dev/ttyUSB1", "px100": "/dev/ttyUSB0"}
    """
    found = {"tray": None, "psu": None, "px100": None}
    for p in serial.tools.list_ports.comports():
        vid = p.vid
        pid = p.pid
        dev = p.device

        if vid == 9025 and pid == 66:
            found["tray"] = dev
        elif vid == 6790 and pid == 29987:
            found["psu"] = dev
        elif vid == 1027 and pid == 24577:
            found["px100"] = dev

    return found
