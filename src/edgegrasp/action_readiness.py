"""Short bounded discovery wait; never sends or retries an action goal."""

import math
import time


def wait_for_readiness(ready, *, timeout_s=0.05, clock=time.monotonic, pause=time.sleep):
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= .05:
        raise ValueError("readiness wait must be in (0, 50 ms]")
    deadline = clock() + timeout_s
    while True:
        if ready():
            return True
        remaining = deadline - clock()
        if remaining <= 0:
            return False
        pause(min(.005, remaining))
