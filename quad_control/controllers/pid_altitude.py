import numpy as np


class PID_alttitude:
    def __init__(self, kp=0.7, ki=0.1, kd=0.1, dt=0.01, feedforward=0.5):
        self.kp_thrust = kp
        self.ki_thrust = ki
        self.kd_thrust = kd
        self.max_thrust = 0.720 
        self.dt = dt
        self.feedforward = feedforward
        self.prev_error = 1000
        self.integral = 0
        self.integral_limit = 1.5
        self.prev_output = 1000
        self.start_pid = False

    def step(self, error, curr):
        if curr < 0.05:
            self.prev_output += 10
            return self.prev_output
        
        # if abs(error) < 0.05:
        # #     self.integral = 0.0
        #     return self.prev_output
            
        
        # PID computations
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        pid_output = self.kp_thrust * error + self.ki_thrust * self.integral + self.kd_thrust * derivative
        
        # Adding feedforward term 
        total_output = (pid_output + self.feedforward) * 0.6
        
        total_output = min(total_output, self.max_thrust)
        total_output = max(total_output, 0)
        output = int((total_output / self.max_thrust) * 1000) + 1000

        # Limit output to avoid exceeding maximum values
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
        self.integral_limit = 1.5
        
    def step(self, error):
        # error = -1 * error
        if error < 0.01 and error > -0.01:
            error = 0
            # self.integral = 0
        error = self.ke * error
        self.integral += error * self.dt * 0.2
        # if self.integral > self.integral_limit:
        #     self.integral = self.integral_limit
        # elif self.integral < -self.integral_limit:
        #     self.integral = -self.integral_limit
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_roll_pitch), -self.max_roll_pitch)
        output = int(((output + self.max_roll_pitch) / (2* self.max_roll_pitch)) * (1000))  + 1000

        self.prev_error = error
        return output