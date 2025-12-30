import os
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
from matplotlib.figure import Figure

from PIL import Image, ImageEnhance


class GraphMaker:
    def __init__(self, receipt_dots=384):
        self.receipt_dots = int(receipt_dots)

        # darkness tuning
        self.contrast = 2.2
        self.sharpness = 1.5
        self.threshold = 180  # lower => darker

    def save_voltage_graph(self, samples, out_dir, capacity_mah):
        """
        samples: list of dicts {"t": seconds, "v": voltage, "mah": ...}
        Produces an 800x600 png, then thumbnails to receipt width, then binarizes darker.
        Returns (path, filename)
        """
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        cap_str = "unknown"
        try:
            if capacity_mah is not None:
                cap_str = "%dmAh" % int(round(float(capacity_mah)))
        except Exception:
            cap_str = "unknown"

        filename = "capacity_%s_%s.png" % (cap_str, ts)
        path = os.path.join(out_dir, filename)

        width_px, height_px = 800, 600
        dpi = 100
        fig = Figure(dpi=dpi, figsize=(width_px / dpi, height_px / dpi))
        ax = fig.add_subplot(111)

        xs = []
        ys = []
        for s in samples or []:
            t = s.get("t")
            v = s.get("v")
            if t is None or v is None:
                continue
            xs.append(float(t))
            ys.append(float(v))

        if xs:
            ax.plot(xs, ys, linewidth=2.0)

        ax.grid(True, linewidth=0.5)
        ax.tick_params(labelsize=8)
        fig.tight_layout()
        fig.savefig(path)

        # thumbnail to receipt width
        try:
            with Image.open(path) as im:
                im = im.convert("L")
                im.thumbnail((self.receipt_dots, 10000), Image.LANCZOS)
                im.save(path)
        except Exception:
            return path, filename

        # darken / binarize
        try:
            with Image.open(path) as im:
                im = im.convert("L")
                im = ImageEnhance.Contrast(im).enhance(float(self.contrast))
                im = ImageEnhance.Sharpness(im).enhance(float(self.sharpness))
                thr = int(self.threshold)
                im = im.point(lambda p: 255 if p > thr else 0, mode="1")
                im.save(path)
        except Exception:
            pass

        return path, filename
