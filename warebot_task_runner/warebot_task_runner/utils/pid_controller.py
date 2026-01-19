"""
PID Controller implementation
"""
import time
import numpy as np


class PID:
    """PID Controller"""
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_limit=1.0, out_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.out_limit = abs(out_limit)
        self.i = 0.0
        self.prev_e = None
        self.prev_t = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None
        self.prev_t = None

    def step(self, e: float) -> float:
        t = time.time()
        dt = 0.0 if self.prev_t is None else max(1e-3, t - self.prev_t)
        p = self.kp * e
        self.i += e * dt
        self.i = float(np.clip(self.i, -self.i_limit, self.i_limit))
        d = 0.0 if self.prev_e is None else self.kd * (e - self.prev_e) / dt
        self.prev_e = e
        self.prev_t = t
        u = p + self.ki * self.i + d
        return float(np.clip(u, -self.out_limit, self.out_limit))