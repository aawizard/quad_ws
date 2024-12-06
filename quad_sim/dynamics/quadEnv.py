#============================================================================================================================================
#Documentation
#--------------
#
#1.1) Set USE_PWM = 1 if you are giving inputs to the step() function as PWM pulses for each BLDC motors (1000us - 2000us)
#1.2) Set USE_PID = 1 if you are using any of the PID controllers for benchmarking purposes. If you want to switch over to any other algorithm in midst of the program execution from PID controller, set USE_PID = 0. If, again you want to use PID controller after your algorithm in midst of the program execution, make sure to call the rstEnv() function before you set the USE_PID flag to 1.
#
#2) The des_xyz(x_des, y_des, z_des) function is used to provide the desired positions, to which the drone must hover to and stabilize. You must call PID_position() function if you are using this.
# 
#3) The step(current_state, input) will calculate the next state of the system, based on the current_state and the input provided, for one time step.
#4) PID_position() calculates the desired roll, pitch and thrust values that are to be followed if the drone has to hover at the desired position.
#5) PID_attitude() calculates the desired rates that the drone must follow, in order to tilt to the commanded attitude
#6) The PID_rate() calculates the desired torques along the x, y, z direction that must be applied by the BLDC motors
#7) The quad_motor_speed() calculates the desired motor speeds based on the thrust, and torque values.
#8) The rstEnv() resets the simulation
#9) The pauseEnv() pauses the simulation
#10) The unpauseEnv() unpauses the simulation
#11) The time_elapsed() displays the simulation time

#============================================================================================================================================
import time
import numpy as np
from math import *
import random


class quadrotor:
    def __init__(self, Ts = 1.0/50.0, USE_PWM=0, USE_PID=0):
        self.Ts = Ts
        self.max_thrust = 3.059 # in N (Kgms-2)
        self.g = 9.81  
        self.m = 0.111
        self.L = 0.06
        self.torque_scaling_factor = 0.00001
        # PID gains for roll and pitch
        self.roll_pid = PIDController(kp=0.75, ki=0.04, kd=0.85)
        self.pitch_pid = PIDController(kp=0.75, ki=0.04, kd=0.85)
        
        #Calculatinf inertia matrix
        self.m_frame = 0.01132
        self.motor_dist = 0.046
        self.m_motor = 0.0005
        self.num_motors = 4
        
        # Inertia of the frame (approximated as a square body)
        self.I_frame = (1/12) * self.m_frame * (self.L**2 + self.L**2)

        # Inertia of motors (point mass at distance L)
        self.I_motors = self.num_motors * (self.m_motor * self.L**2)

        # Total inertia of the body
        self.I_body = self.I_frame + self.I_motors

        # Now calculate diagonal components I_xx, I_yy, I_zz (approximating all arms and motors along the axes)
        self.I_xx = self.I_body + 4 * (1/12) * self.m_motor * self.L**2  # Inertia along x-axis for each arm
        self.I_yy = self.I_xx  # Assuming symmetry
        self.I_zz = self.I_body + 4 * (self.m_motor * self.L**2)  # Considering motors are on the edge

        # Inertia tensor
        self.I = np.diag([self.I_xx, self.I_yy, self.I_zz])
        self.I_inv = np.linalg.inv(self.I)  # Inverse of inertia tensor
        
        self.r_dot = 0.0
        
        self.state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.o = 0.0
        
        self.rstFlag = 0
        self.pauseFlag = 0
        
        self.time_elapse = 0.0
        
        self.USE_PID = USE_PID
        self.USE_PWM = USE_PWM
        if(self.USE_PWM == 1 and self.USE_PID == 0):
            self.input_vector = [1000, 1000, 1000, 1000]
            print(self.input_vector)
        else:
            self.input_vector = [0, 0, 0, 0]
    
    def des_xyz(self, x_des=1.0, y_des=1.0, z_des=1.0):
        self.x_des = x_des
        self.y_des = y_des
        self.z_des = z_des
        
    def step(self ,state, input_vector):
        # print(input_vector)
        #self.state = [phi, theta, psi, p, q, r, x_dot, y_dot, z_dot, x, y, z]
        if(self.USE_PWM == 1 and self.USE_PID == 0):
            if(input_vector[0] < 1000.0):   
                input_vector[0] = 1000.0
                # print("Warning!! PWM thrust less than 1000.0")
            if(input_vector[1] < 1000.0):
                input_vector[1] = 1500.0
                print("Warning!! PWM roll less than 1000.0")
            if(input_vector[2] < 1000.0):
                input_vector[2] = 1500.0
                print("Warning!! PWM pitch less than 1000.0")
            if(input_vector[3] < 1000.0):
                input_vector[3] = 1500.0
                print("Warning!! PWM yaw less than 1000.0")
                
            phi, theta, psi, p, q, r, x_dot, y_dot, z_dot, x, y, z = self.state
            
            # Convert PWM inputs to target angles
            roll_target = (((input_vector[1] - 1050.0) / 900) * 110.0) - 55.0
            pitch_target = (((input_vector[2] - 1050.0) / 900) * 110.0) - 55.0
            thrust = (((input_vector[0] - 1000.0) / 1000) * self.max_thrust)
            thrust_gf = self.rotationMatrixBFtoGF(phi, theta, psi) @ np.array([0, 0, thrust])
            roll_error = np.radians(roll_target) -( phi)
            roll_control = self.roll_pid.update(roll_error)
            # Time step
            dt = 0.01  # For example, 10 ms
            
            # PID control for pitch
            pitch_error = np.radians(pitch_target) - theta
            pitch_control = self.pitch_pid.update(pitch_error)
            
            # Convert roll and pitch control efforts to radians for angular rate update
            roll_control_rad = (roll_control)
            pitch_control_rad = (pitch_control)
            
            
            
            # Update angular rates based on PID output
            
            p = roll_control_rad
            q = pitch_control_rad
            r += 0  # Assuming no yaw input for now
          
            # # Current angular velocity vector
            omega_body = np.array([p, q, r])
            tau = np.array([roll_control, pitch_control, 0]) * self.torque_scaling_factor  # Assuming zero yaw control

            # # # Compute angular acceleration
            omega_dot = self.I_inv @ (tau - np.cross(omega_body, self.I @ omega_body))

            # # # Update angular velocities
            omega_body += omega_dot * dt
            # p, q, r = omega_body

            if random.uniform(0, 1) < 0.4:
                p += random.uniform(-0.3, 0.3)
                q += random.uniform(-0.3, 0.3)
            
            # Update angles and other state variables
            phi += p * dt
            theta += q * dt
            psi += r * dt
            # print(thrust_gf)
            z_dot += (thrust_gf[2]/self.m - self.g)  * dt  # Net force in z direction
            z += z_dot * dt  # Update z position

            # Check if z < 0
            if z < 0:
                z = 0.0
                z_dot = 0.0
                x_dot = 0.0
                y_dot = 0.0
                # print("Warning!! Z position is below 0. Quadcopter will not move.")
            else:
                x_dot -= thrust_gf[0] * dt
                y_dot -= thrust_gf[1] * dt
                x += x_dot * dt 
                y += y_dot * dt
            # Update the state
            self.state = [phi, theta, psi, p, q, r, x_dot, y_dot, z_dot, x, y, z]
            return self.state
                                
        else:
            return self.state
        
    def rotateGFtoBF(self, X, Y, Z, PHI, THETA, PSI):
        X_ = cos(PSI)*cos(THETA)*X + sin(PSI)*cos(THETA)*Y - sin(THETA)*Z
        Y_ = (cos(PSI)*sin(PHI)*sin(THETA) - cos(PHI)*sin(PSI))*X + (sin(PHI)*sin(PSI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(THETA)*sin(PHI))*Z
        Z_ = (cos(PHI)*cos(PSI)*sin(THETA) + sin(PHI)*sin(PSI))*X + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Y + (cos(PHI)*cos(THETA))*Z
        return (X_, Y_, Z_)
    
    def rotateBFtoGF(self, X, Y, Z, PHI, THETA, PSI):
        X_ = cos(PSI)*cos(THETA)*X + (cos(PSI)*sin(PHI)*sin(THETA) - sin(PSI)*cos(PHI))*Y + (cos(PHI)*cos(PSI)*sin(THETA) + sin(PHI)*sin(PSI))*Z
        Y_ = sin(PSI)*cos(THETA)*X + (sin(PHI)*sin(PSI)*sin(THETA) + cos(PHI)*cos(PSI))*Y + (cos(PHI)*sin(PSI)*sin(THETA) - cos(PSI)*sin(PHI))*Z
        Z_ = -sin(THETA)*X + cos(THETA)*sin(PHI)*Y + cos(PHI)*cos(THETA)*Z
        return (X_, Y_, Z_)
    
    def rotationMatrixBFtoGF(self,phi, theta, psi):
    # Rotation matrix from Body Frame (BF) to Global Frame (GF)
        R = [
            [
                cos(psi) * cos(theta),
                sin(psi) * cos(theta),
                -sin(theta)
            ],
            [
                cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi),
                sin(phi) * sin(psi) * sin(theta) + cos(phi) * cos(psi),
                cos(theta) * sin(phi)
            ],
            [
                cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi),
                cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi),
                cos(phi) * cos(theta)
            ]
        ]
        return R
            
        
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.d =0.0

    def update(self, error):
        # error = error * 100
        # if abs(error )< 3.0:
        #     self.integral = 0.0
        #     error = 0.0
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error
        self.integral = max(-1.0, min(1.0, self.integral))
        i = self.ki * self.integral

        # Derivative term
        # d = self.kd * (error - self.previous_error)
        alpha = 0.1  # Adjust between 0.1 and 0.3 based on response
        d = self.kd * (alpha * (error - self.previous_error) + (1 - alpha) * self.d)
        self.d = d
        self.previous_error = error
        output = (p + i + d)
        output = max(-1.0, min(1.0, output))

        # PID output
        return output