import tkinter as tk
from tkinter import ttk, messagebox

from devices.ports import auto_detect_ports


class App(tk.Tk):
    def __init__(self, tray, runner, ports_detected):
        super().__init__()
        self.title("Automated Battery Tester")
        self.geometry("900x520")

        self.tray = tray
        self.runner = runner

        # UI vars
        self.tray_port = tk.StringVar(value=ports_detected.get("tray") or "")
        self.psu_port = tk.StringVar(value=ports_detected.get("psu") or "")
        self.px_port = tk.StringVar(value=ports_detected.get("px100") or "")

        self.charge_enabled = tk.BooleanVar(value=True)
        self.test_enabled = tk.BooleanVar(value=True)

        self.test_current_a = tk.StringVar(value="1.00")
        self.cutoff_v = tk.StringVar(value="3.00")

        # compact styling
        style = ttk.Style(self)
        style.configure("TLabel", font=("Sans", 10))
        style.configure("TButton", font=("Sans", 11, "bold"), padding=(6, 4))
        style.configure("TCheckbutton", font=("Sans", 10))
        style.configure("TLabelframe.Label", font=("Sans", 10))

        self._build_ui()
        self.after(100, self._ui_pump)

    def _build_ui(self):
        root = ttk.Frame(self, padding=6)
        root.pack(fill="both", expand=True)

        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True)

        tab_run = ttk.Frame(nb, padding=6)
        tab_conn = ttk.Frame(nb, padding=6)
        tab_log = ttk.Frame(nb, padding=6)

        nb.add(tab_run, text="Run")
        nb.add(tab_conn, text="Connections")
        nb.add(tab_log, text="Log")

        # ---- Run tab ----
        left = ttk.Frame(tab_run)
        right = ttk.Frame(tab_run)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        right.grid(row=0, column=1, sticky="nsew")

        tab_run.columnconfigure(0, weight=1)
        tab_run.columnconfigure(1, weight=2)
        tab_run.rowconfigure(0, weight=1)

        lf = ttk.LabelFrame(left, text="Workflow", padding=6)
        lf.pack(fill="x")

        ttk.Button(lf, text="LOAD TRAY", command=self._load_tray).pack(fill="x", pady=3)
        ttk.Button(lf, text="START AUTO TEST", command=self._start_auto).pack(fill="x", pady=3)
        ttk.Button(lf, text="STOP", command=self._stop).pack(fill="x", pady=3)
        ttk.Button(lf, text="TEST PRINTER", command=self._printer_test).pack(fill="x", pady=(8, 0))

        opt = ttk.LabelFrame(left, text="Options", padding=6)
        opt.pack(fill="x", pady=(6, 0))

        ttk.Checkbutton(opt, text="Charge", variable=self.charge_enabled).pack(anchor="w")
        ttk.Checkbutton(opt, text="Capacity test", variable=self.test_enabled).pack(anchor="w")

        row = ttk.Frame(opt)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="Test I(A):").pack(side="left")
        ttk.Entry(row, textvariable=self.test_current_a, width=6).pack(side="left", padx=(4, 10))
        ttk.Label(row, text="Cutoff V:").pack(side="left")
        ttk.Entry(row, textvariable=self.cutoff_v, width=6).pack(side="left", padx=4)

        stat = ttk.LabelFrame(right, text="Status", padding=6)
        stat.pack(fill="both", expand=True)

        self.lbl_psu = tk.StringVar(value="PSU: --")
        self.lbl_px = tk.StringVar(value="PX100: --")
        self.lbl_tray = tk.StringVar(value="Tray: --")
        self.lbl_v = tk.StringVar(value="V: --")
        self.lbl_i = tk.StringVar(value="I: --")
        self.lbl_cap = tk.StringVar(value="Cap: --")

        def srow(r, label, var):
            ttk.Label(stat, text=label).grid(row=r, column=0, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(stat, textvariable=var, font=("Sans", 11, "bold")).grid(row=r, column=1, sticky="w", pady=2)

        srow(0, "Tray:", self.lbl_tray)
        srow(1, "PSU:", self.lbl_psu)
        srow(2, "PX100:", self.lbl_px)
        srow(3, "Voltage:", self.lbl_v)
        srow(4, "Current:", self.lbl_i)
        srow(5, "Capacity:", self.lbl_cap)

        # ---- Connections tab ----
        conn = ttk.LabelFrame(tab_conn, text="Ports", padding=6)
        conn.pack(fill="x")

        ttk.Button(conn, text="Auto-detect", command=self._autodetect).pack(fill="x", pady=(0, 6))

        def crow(title, var):
            fr = ttk.Frame(conn)
            fr.pack(fill="x", pady=2)
            ttk.Label(fr, text=title, width=8).pack(side="left")
            ttk.Entry(fr, textvariable=var).pack(side="left", fill="x", expand=True)

        crow("Tray", self.tray_port)
        crow("PSU", self.psu_port)
        crow("PX100", self.px_port)

        btns = ttk.Frame(conn)
        btns.pack(fill="x", pady=(6, 0))
        ttk.Button(btns, text="Connect All", command=self._connect_all).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(btns, text="Disconnect All", command=self._disconnect_all).pack(side="left", expand=True, fill="x")

        # ---- Log tab ----
        logf = ttk.LabelFrame(tab_log, text="Log", padding=6)
        logf.pack(fill="both", expand=True)

        self.log = tk.Text(logf, wrap="word", font=("Sans", 10))
        yscroll = ttk.Scrollbar(logf, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=yscroll.set)
        self.log.pack(side="left", fill="both", expand=True)
        yscroll.pack(side="right", fill="y")
        self.log.configure(state="disabled")

    # ---------- UI actions ----------
    def _connect_all(self):
        tray = self.tray_port.get().strip()
        psu = self.psu_port.get().strip()
        px = self.px_port.get().strip()

        if not tray and not psu and not px:
            messagebox.showwarning("Ports", "No ports set. Click Auto-detect first.")
            return

        self._log_local("Connect All clicked: tray=%r psu=%r px=%r" % (tray, psu, px))
        self.runner.connect_all(tray, psu, px)

    def _disconnect_all(self):
        self.runner.disconnect_all()

    def _autodetect(self):
        found = auto_detect_ports()
        if found.get("tray"):
            self.tray_port.set(found["tray"])
        if found.get("psu"):
            self.psu_port.set(found["psu"])
        if found.get("px100"):
            self.px_port.set(found["px100"])

    def _load_tray(self):
        self.runner.load_tray()

    def _stop(self):
        self.runner.stop()

    def _printer_test(self):
        self.runner.printer_test_dummy()

    def _start_auto(self):
        try:
            test_i = float(self.test_current_a.get().strip())
            cutoff = float(self.cutoff_v.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid", "Enter numeric test current and cutoff voltage.")
            return

        self.runner.start_auto(
            charge_enabled=self.charge_enabled.get(),
            test_enabled=self.test_enabled.get(),
            test_current_a=test_i,
            cutoff_v=cutoff,
        )
    def _log_local(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ---------- Pump updates ----------
    def _ui_pump(self):
        # logs
        try:
            while True:
                line = self.runner.logbus.q.get_nowait()
                self.log.configure(state="normal")
                self.log.insert("end", line + "\n")
                self.log.see("end")
                self.log.configure(state="disabled")
        except Exception:
            pass

        # status
        st = self.runner.status
        self.lbl_tray.set("Connected" if st.get("tray_connected") else "Disconnected")
        self.lbl_psu.set("Connected" if st.get("psu_connected") else "Disconnected")
        self.lbl_px.set("Connected" if st.get("px_connected") else "Disconnected")

        v = st.get("px_voltage_v")
        i = st.get("px_current_a")
        cap = st.get("px_cap_mah")

        self.lbl_v.set("--" if v is None else "%.3f V" % float(v))
        self.lbl_i.set("--" if i is None else "%.3f A" % float(i))
        self.lbl_cap.set("--" if cap is None else "%.0f mAh" % float(cap))

        self.after(200, self._ui_pump)
