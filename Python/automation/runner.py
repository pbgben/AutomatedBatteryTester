import os
import threading
import time
import queue

from util.log import LogBus


def safe_float(x):
    try:
        return float(x)
    except Exception:
        return None


class AutomationRunner:
    """
    Background automation state machine.

    Public API:
      - connect_all(tray_port, psu_port, px_port)
      - load_tray()
      - start_auto(charge_enabled, test_enabled, test_current_a, cutoff_v)
      - stop()
      - printer_test_dummy()

    UI reads:
      - self.logbus.q
      - self.status dict
    """
    def __init__(self, tray, psu_factory, px_factory, printer, graphs, output_dir):
        self.tray = tray
        self.psu_factory = psu_factory
        self.px_factory = px_factory
        self.printer = printer
        self.graphs = graphs
        self.output_dir = output_dir

        self.logbus = LogBus()
        self.status = {
            "psu_connected": False,
            "px_connected": False,
            "tray_connected": False,
            "psu_out": None,
            "psu_current_a": None,
            "px_voltage_v": None,
            "px_current_a": None,
            "px_cap_mah": None,
        }

        self.psu = None
        self.px = None

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._cmdq = queue.Queue()
        self._stop_evt = threading.Event()
        self._auto_running = False

        # rules
        self.min_cell_voltage_v = 2.0
        self.charge_cutoff_a = 0.10
        self.charge_stabilize_s = 10.0

        # per-cell timing
        self._cell_charge_start_ts = None
        self._cell_charge_end_ts = None
        self._cell_discharge_start_ts = None
        self._cell_discharge_end_ts = None
        self._cell_start_voltage = None

        # test capture
        self._test_running = False
        self._test_samples = []
        self._test_t0 = None
        self._test_cutoff_v = None

        # params
        self._charge_enabled = True
        self._test_enabled = True
        self._test_current_a = 1.0
        self._cutoff_v = 3.0

        # tray empty heuristic
        self._tray_empty_hint = False

        self._thread.start()

    # ---------- UI commands ----------
    def connect_all(self, tray_port, psu_port, px_port):
        self._cmdq.put(("CONNECT_ALL", {"tray": tray_port, "psu": psu_port, "px": px_port}))

    def disconnect_all(self):
        self._cmdq.put(("DISCONNECT_ALL", {}))

    def load_tray(self):
        self._cmdq.put(("LOAD_TRAY", {}))

    def start_auto(self, charge_enabled, test_enabled, test_current_a, cutoff_v):
        self._cmdq.put(("START_AUTO", {
            "charge": bool(charge_enabled),
            "test": bool(test_enabled),
            "test_current_a": float(test_current_a),
            "cutoff_v": float(cutoff_v),
        }))

    def stop(self):
        self._cmdq.put(("STOP", {}))
        self._stop_evt.set()

    def printer_test_dummy(self):
        self._cmdq.put(("PRINTER_TEST", {}))

    # ---------- internal helpers ----------
    def _set_status(self, **kwargs):
        self.status.update(kwargs)

    def _log(self, msg):
        self.logbus.log(msg)

    def _poll_devices(self):
        # tray
        self._set_status(tray_connected=self.tray.is_connected())

        # psu
        if self.psu and self.psu.connected:
            out_en = self.psu.read_output_enabled()
            cur_a = self.psu.read_current_a()
            self._set_status(psu_connected=True, psu_out=out_en, psu_current_a=cur_a)
        else:
            self._set_status(psu_connected=False, psu_out=None, psu_current_a=None)

        # px
        if self.px and self.px.connected:
            row = self.px.read_all()
            v = safe_float(row.get("voltage"))
            i = safe_float(row.get("current"))
            cap_mah = self._extract_mah(row)
            self._set_status(px_connected=True, px_voltage_v=v, px_current_a=i, px_cap_mah=cap_mah)

            if self._test_running and self._test_t0 is not None:
                self._record_sample(v=v, mah=cap_mah)
                if self._test_cutoff_v is not None and v is not None and v <= float(self._test_cutoff_v):
                    self._log("AUTO: cutoff reached (V=%.3f <= %.3f) stopping test" % (v, float(self._test_cutoff_v)))
                    try:
                        self.px.set_load_enabled(False)
                    except Exception:
                        pass
                    self._test_running = False
                    self._cell_discharge_end_ts = time.time()
                    self._finish_and_print()
        else:
            self._set_status(px_connected=False, px_voltage_v=None, px_current_a=None, px_cap_mah=None)

    def _extract_mah(self, row):
        if not row:
            return None
        v = safe_float(row.get("cap_mah"))
        if v is not None:
            return v
        v = safe_float(row.get("cap_ah"))
        if v is not None:
            return v * 1000.0
        return None

    def _record_sample(self, v, mah):
        if v is None and mah is None:
            return
        t = time.time() - float(self._test_t0)
        self._test_samples.append({"t": t, "v": v, "mah": mah})

    def _force_outputs_off(self):
        try:
            if self.psu and self.psu.connected:
                self.psu.set_output_enabled(False)
        except Exception:
            pass
        try:
            if self.px and self.px.connected:
                self.px.set_load_enabled(False)
        except Exception:
            pass
        try:
            if self.tray and self.tray.is_connected():
                self.tray.send("DISCONNECT")
        except Exception:
            pass

    def _finish_and_print(self):
        # capacity from last sample with mah
        cap = None
        for s in reversed(self._test_samples):
            cap = safe_float(s.get("mah"))
            if cap is not None:
                break

        charge_s = None
        if self._cell_charge_start_ts and self._cell_charge_end_ts:
            charge_s = self._cell_charge_end_ts - self._cell_charge_start_ts

        discharge_s = None
        if self._cell_discharge_start_ts and self._cell_discharge_end_ts:
            discharge_s = self._cell_discharge_end_ts - self._cell_discharge_start_ts

        graph_path = None
        try:
            graph_path, _ = self.graphs.save_voltage_graph(
                self._test_samples, out_dir=self.output_dir, capacity_mah=cap
            )
            self._log("AUTO: saved graph %s" % graph_path)
        except Exception as e:
            self._log("AUTO: graph failed: %s" % e)

        # print
        try:
            self._log("AUTO: printing receipt...")
            self.printer.print_result(
                graph_path=graph_path,
                capacity_mah=cap,
                test_current_a=self._test_current_a,
                cutoff_v=self._cutoff_v,
                charge_s=charge_s,
                discharge_s=discharge_s,
                start_v=self._cell_start_voltage,
            )
            self._log("AUTO: printed")
        except Exception as e:
            self._log("AUTO: print failed: %s" % e)

    # ---------- command handling ----------
    def _do_connect_all(self, tray_port, psu_port, px_port):
        # tray
        if tray_port:
            try:
                self.tray.connect(tray_port, 9600)
                self._log("Tray connected: %s" % tray_port)
            except Exception as e:
                self._log("Tray connect failed: %s" % e)

        # psu
        if psu_port:
            try:
                self.psu = self.psu_factory(psu_port)
                if self.psu.connect():
                    self._log("PSU connected: %s" % psu_port)
                else:
                    self._log("PSU connect failed")
                    self.psu = None
            except Exception as e:
                self._log("PSU connect error: %s" % e)
                self.psu = None

        # px
        if px_port:
            try:
                self.px = self.px_factory(px_port)
                if self.px.connect():
                    self._log("PX100 connected: %s" % px_port)
                else:
                    self._log("PX100 connect failed")
                    self.px = None
            except Exception as e:
                self._log("PX100 connect error: %s" % e)
                self.px = None
        self._poll_devices()

    def _do_disconnect_all(self):
        self._auto_running = False
        self._force_outputs_off()

        try:
            if self.psu:
                self.psu.disconnect()
        except Exception:
            pass
        self.psu = None

        try:
            if self.px:
                self.px.disconnect()
        except Exception:
            pass
        self.px = None

        try:
            self.tray.disconnect()
        except Exception:
            pass

        self._log("Disconnected all")

    def _do_load_tray(self):
        if not self.tray.is_connected():
            self._log("LOAD: tray not connected")
            return
        self._log("LOAD: HOME")
        self.tray.send("HOME")
        time.sleep(0.25)
        self._log("LOAD: RECONNECT")
        self.tray.send("RECONNECT")

    def _do_printer_test(self):
        # Generate a dummy curve quickly using graph maker directly
        # (no need to simulate full PX)
        dummy = []
        total_s = 45 * 60
        step_s = 10
        cutoff = 3.0
        start_v = 4.15
        n = total_s // step_s
        for i in range(int(n) + 1):
            t = i * step_s
            frac = float(t) / float(total_s)
            v = start_v - 0.2 * min(1.0, frac / 0.1)  # quick sag
            v -= 0.4 * max(0.0, min(1.0, (frac - 0.1) / 0.75))
            v -= 0.6 * max(0.0, min(1.0, (frac - 0.85) / 0.15))
            if v < cutoff:
                v = cutoff
            dummy.append({"t": t, "v": v, "mah": 2500.0 * frac})

        cap = 3100.0
        graph_path = None
        try:
            graph_path, _ = self.graphs.save_voltage_graph(dummy, out_dir=self.output_dir, capacity_mah=cap)
        except Exception as e:
            self._log("Printer test graph failed: %s" % e)
            graph_path = None

        try:
            self.printer.print_test_page(graph_path)
            self._log("Printer test sent")
        except Exception as e:
            self._log("Printer test failed: %s" % e)

    # ---------- automation cycle ----------
    def _do_start_auto(self, charge, test, test_current_a, cutoff_v):
        if not self.tray.is_connected():
            self._log("AUTO: tray not connected")
            return
        if charge and (not self.psu or not self.psu.connected):
            self._log("AUTO: charge enabled but PSU not connected")
            return
        if (charge or test) and (not self.px or not self.px.connected):
            self._log("AUTO: PX100 required (voltage check). Not connected.")
            return
        if (not charge) and (not test):
            self._log("AUTO: nothing enabled (charge/test unchecked)")
            return

        self._charge_enabled = bool(charge)
        self._test_enabled = bool(test)
        self._test_current_a = float(test_current_a)
        self._cutoff_v = float(cutoff_v)

        self._auto_running = True
        self._tray_empty_hint = False
        self._log("AUTO: started (charge=%s test=%s)" % (charge, test))

        # main loop per cell
        while self._auto_running and (not self._stop_evt.is_set()):
            # 1) check voltage gate
            if not self.px or not self.px.connected:
                self._log("AUTO: PX disconnected. Stopping.")
                break

            try:
                self.px.set_load_enabled(False)
            except Exception:
                pass
            time.sleep(0.2)

            v = self.px.read_voltage_v()
            self._cell_start_voltage = v

            if v is None:
                self._log("AUTO: voltage read failed. Treat as empty -> stopping.")
                break

            if v <= float(self.min_cell_voltage_v):
                self._log("AUTO: voltage too low (%.3fV <= %.2fV). Skipping cell." % (v, float(self.min_cell_voltage_v)))
                # move next
                if not self._move_next_cell():
                    break
                continue

            self._log("AUTO: cell voltage OK (%.3fV)" % v)

            # 2) charge
            if self._charge_enabled:
                if not self._do_charge_cycle():
                    break

            # 3) test
            if self._test_enabled:
                if not self._do_capacity_test_cycle():
                    break

            # 4) disconnect contacts between cells
            try:
                self.tray.send("DISCONNECT")
            except Exception:
                pass

            # 5) next
            if not self._move_next_cell():
                break

        self._auto_running = False
        self._force_outputs_off()
        self._log("AUTO: done")

    def _do_charge_cycle(self):
        # PSU ON until current <= cutoff for stabilize seconds
        if not self.psu or not self.psu.connected:
            self._log("AUTO: PSU missing during charge")
            return False

        self._cell_charge_start_ts = time.time()
        self._cell_charge_end_ts = None

        self._log("AUTO: PSU ON (charging)")
        if not self.psu.set_output_enabled(True):
            self._log("AUTO: PSU ON failed")
            return False

        low_since = None
        while self._auto_running and (not self._stop_evt.is_set()):
            time.sleep(0.4)
            cur = self.psu.read_current_a()
            out_en = self.psu.read_output_enabled()

            # update UI status
            self._set_status(psu_current_a=cur, psu_out=out_en)

            if out_en == 0:
                self._log("AUTO: PSU output OFF unexpectedly")
                return False

            if cur is None:
                continue

            if cur <= float(self.charge_cutoff_a):
                if low_since is None:
                    low_since = time.time()
                    self._log("AUTO: current low, waiting %ds stabilize..." % int(self.charge_stabilize_s))
                elif (time.time() - low_since) >= float(self.charge_stabilize_s):
                    break
            else:
                low_since = None

        self._log("AUTO: PSU OFF (charge complete)")
        try:
            self.psu.set_output_enabled(False)
        except Exception:
            pass
        self._cell_charge_end_ts = time.time()
        return True

    def _do_capacity_test_cycle(self):
        # Always PSU OFF
        try:
            if self.psu and self.psu.connected:
                self.psu.set_output_enabled(False)
        except Exception:
            pass

        # Reset tester, start test, capture samples until cutoff hit
        self._cell_discharge_start_ts = time.time()
        self._cell_discharge_end_ts = None

        self._test_running = True
        self._test_samples = []
        self._test_t0 = time.time()
        self._test_cutoff_v = float(self._cutoff_v)

        self._log("AUTO: PX100 reset")
        try:
            self.px.reset()
        except Exception:
            pass
        time.sleep(0.25)

        self._log("AUTO: PX100 start test (I=%.2fA Vcut=%.2fV)" % (self._test_current_a, self._cutoff_v))
        ok = self.px.start_capacity_test(current_a=self._test_current_a, cutoff_v=self._cutoff_v)
        if not ok:
            self._log("AUTO: PX start failed")
            self._test_running = False
            return False

        # wait until _poll_devices detects cutoff and finishes
        while self._auto_running and self._test_running and (not self._stop_evt.is_set()):
            time.sleep(0.2)
            # device polling happens in outer loop
            self._poll_devices()

        return True

    def _move_next_cell(self):
        if not self.tray.is_connected():
            self._log("AUTO: tray disconnected")
            return False

        self._log("AUTO: NEXT cell")
        self.tray.send("NEXT")
        time.sleep(0.25)
        self.tray.send("RECONNECT")
        time.sleep(1.0)

        # heuristic: if detectClosed=0 and voltage fails repeatedly, we will stop next cycle
        return True

    # ---------- main thread loop ----------
    def _loop(self):
        last_poll = 0.0
        while not self._stop_evt.is_set():
            # periodic poll
            now = time.time()
            if (now - last_poll) >= 0.4:
                last_poll = now
                self._poll_devices()

                # also consume tray lines for hints
                try:
                    while True:
                        line = self.tray.rx_lines.get_nowait()
                        u = line.upper()
                        if any(tok in u for tok in ("NO_MORE", "NO MORE", "EMPTY", "OUT OF CELLS", "END")):
                            self._tray_empty_hint = True
                except queue.Empty:
                    pass

            # commands
            try:
                cmd, payload = self._cmdq.get_nowait()
                self._log("CMD: %s %s" % (cmd, payload))
            except queue.Empty:
                time.sleep(0.02)
                continue

            if cmd == "CONNECT_ALL":
                self._do_connect_all(payload.get("tray"), payload.get("psu"), payload.get("px"))
            elif cmd == "DISCONNECT_ALL":
                self._do_disconnect_all()
            elif cmd == "LOAD_TRAY":
                self._do_load_tray()
            elif cmd == "PRINTER_TEST":
                self._do_printer_test()
            elif cmd == "START_AUTO":
                # run blocking in this background thread
                self._do_start_auto(
                    payload.get("charge", True),
                    payload.get("test", True),
                    payload.get("test_current_a", 1.0),
                    payload.get("cutoff_v", 3.0),
                )
            elif cmd == "STOP":
                self._auto_running = False
                self._force_outputs_off()
