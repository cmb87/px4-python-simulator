import numpy as np

# -------------------------------------------------------------
# PID controllers
# -------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, limit=np.inf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.int = 0
        self.prev = 0

    def reset(self):
        self.int = 0
        self.prev = 0

    def update(self, error, dt):
        self.int += error * dt
        d = (error - self.prev) / dt
        self.prev = error
        y = self.kp * error + self.ki * self.int + self.kd * d
        return np.clip(y, -self.limit, self.limit)