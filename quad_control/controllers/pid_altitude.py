import numpy as np


class PID_alttitude:
    def __init__(self, kp=0.7, ki=0.1, kd=0.1, dt=0.01):
        self.kp_thrust = kp
        self.ki_thrust = ki
        self.kd_thrust = kd
        self.max_thrust = 0.720 
        self.dt = dt
        self.prev_error = 1000
        self.integral = 0
        self.prev_output = 1000
        self.start_pid = False

    def step(self, error):
        # if abs(error) < 0.01:
        #     return self.prev_output
        # if not self.start_pid and error > 0.05:
        #     output = self.prev_output + 2
        #     output = min(output, 1950)
        #     self.prev_output = output
        #     return self.prev_output
        # self.start_pid = True 
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = min(self.kp_thrust * error + self.ki_thrust * self.integral + self.kd_thrust * derivative, self.max_thrust)
        # output = self.kp_thrust * error + self.ki_thrust * self.integral + self.kd_thrust * derivative
        output = max(output, 0)
        output = int((output/self.max_thrust) * 1000) + 1000
        # output += self.prev_output
        output = min(output, 1950)
        self.prev_error = error
        self.prev_output = output
        return output

class PID_roll_pitch:
    def __init__(self, kp=0.7, ki=0.1, kd=0.1, dt=0.01):
        self.ke = 15.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_roll_pitch = 55
        self.dt = dt
        self.prev_error = 0
        self.integral = 0
        
    def step(self, error):
        # error = -1 * error
        if error < 0.01 and error > -0.01:
            error = 0
        error = self.ke * error
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_roll_pitch), -self.max_roll_pitch)
        output = int(((output + self.max_roll_pitch) / (2* self.max_roll_pitch)) * (1000))  + 1000

        self.prev_error = error
        return output