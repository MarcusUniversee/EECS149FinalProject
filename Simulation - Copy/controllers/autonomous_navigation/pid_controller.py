import math
from dataclasses import dataclass

def wrap_angle(a):
    # Wrap to [-pi, pi]
    return (a + math.pi) % (2.0 * math.pi) - math.pi

@dataclass
class PID:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    integral: float = 0.0
    prev_error: float = 0.0
    integral_limit: float = 1.0  # anti-windup

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        # Proportional
        p = self.kp * error

        # Integral
        self.integral += error * dt
        # Clamp integral to avoid windup
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        i = self.ki * self.integral

        # Derivative
        d = self.kd * (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error

        return p + i + d