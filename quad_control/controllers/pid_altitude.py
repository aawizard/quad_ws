import numpy as np


class PID_alttitude:
    def __init__(self, kp=0.7, ki=0.1, kd=0.1, dt=0.01):
        self.kp_thrust = kp
        self.ki_thrust = ki
        self.kd_thrust = kd
        self.max_thrust = 0.520
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def step(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = min(self.kp_thrust * error + self.ki_thrust * self.integral + self.kd_thrust * derivative, self.max_thrust)
        output = max(output, 0)
        output = int((output/self.max_thrust) * 1000) + 1000
        self.prev_error = error
        return output