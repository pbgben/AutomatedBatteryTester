import queue
import time
from datetime import datetime

class LogBus:
    """
    Thread-safe log message bus; UI consumes from queue.
    """
    def __init__(self):
        self.q = queue.Queue()

    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.q.put("[%s] %s" % (ts, msg))
