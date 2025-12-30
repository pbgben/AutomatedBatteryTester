def fmt_duration(seconds):
    if seconds is None:
        return "--:--"
    try:
        s = int(round(float(seconds)))
    except Exception:
        return "--:--"
    if s < 0:
        s = 0
    m = s // 60
    r = s % 60
    if m < 100:
        return "%02d:%02d" % (m, r)
    h = m // 60
    m2 = m % 60
    return "%d:%02d:%02d" % (h, m2, r)
