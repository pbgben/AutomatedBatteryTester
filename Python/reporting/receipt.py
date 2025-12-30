from datetime import datetime

from escpos.printer import Network

from util.timefmt import fmt_duration


class ReceiptPrinter:
    def __init__(self, ip, port=9100, receipt_dots=384):
        self.ip = ip
        self.port = int(port)
        self.receipt_dots = int(receipt_dots)

    def print_result(self, graph_path, capacity_mah, test_current_a, cutoff_v,
                     charge_s, discharge_s, start_v):
        """
        Print a receipt with summary and the graph image.
        """
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        p = Network(self.ip, port=self.port, timeout=10)

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

        if test_current_a is None or cutoff_v is None:
            p.text("I: --.-A  Vcut: --.--V\n")
        else:
            p.text("I: %.2fA  Vcut: %.2fV\n" % (float(test_current_a), float(cutoff_v)))

        p.text("Tchg:%s  Tdis:%s\n" % (fmt_duration(charge_s), fmt_duration(discharge_s)))

        if start_v is None:
            p.text("Vstart: --.--V\n")
        else:
            p.text("Vstart: %.2fV\n" % float(start_v))

        p.text("\n")
        if graph_path:
            p.set(align="center")
            p.image(graph_path)
            p.text("\n")

        p.cut()

    def print_test_page(self, graph_path):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        p = Network(self.ip, port=self.port, timeout=10)
        p.set(align="center", bold=True, width=2, height=2)
        p.text("TEST RECEIPT\n")
        p.set(align="center", bold=False, width=1, height=1)
        p.text(ts + "\n\n")
        if graph_path:
            p.image(graph_path)
            p.text("\n")
        p.cut()
