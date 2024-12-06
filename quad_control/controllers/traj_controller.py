import numpy as np
from scipy.spatial.transform import Rotation as R
from controllers.traj_generator import TrajectoryGenerator

class TrajectoryController:
    """
    A class to control the trajectory and orientation of a quadrotor drone using PID-like control laws.

    Attributes:
        m (float): Mass of the quadrotor.
        Kp (ndarray): Proportional gain matrix for position control.
        Kv (ndarray): Derivative gain matrix for velocity control.
        Kr (ndarray): Proportional gain matrix for orientation control.
        Kw (ndarray): Derivative gain matrix for angular velocity control.
        t (float): Current time in the simulation.
        prev_pos (ndarray): Previous position of the quadrotor.
        prev_quat (ndarray): Previous orientation of the quadrotor in quaternion format.
    """
    def __init__(self, m, Kp, Kv, Kr, Kw):
        """
        Initialize the trajectory controller with necessary parameters.

        Args:
            m (float): Mass of the quadrotor.
            Kp (ndarray): Proportional gain matrix for position control.
            Kv (ndarray): Derivative gain matrix for velocity control.
            Kr (ndarray): Proportional gain matrix for orientation control.
            Kw (ndarray): Derivative gain matrix for angular velocity control.
        """
        self.m = m
        self.Kp = Kp
        self.Kv = Kv
        self.Kr = Kr
        self.Kw = Kw
        self.t = 0.0
        self.Kp_scale_factor = 0.2
        self.prev_pos = np.zeros(3)
        self.prev_quat = np.array([1, 0, 0, 0])  # Identity quaternion
        self.Fdes = np.zeros(3)  # Desired force vector
        self.u1 = 0.0  # Total thrust
        self.Rdes = np.eye(3)  # Desired rotation matrix
        self.error_pos = np.zeros(3)
        self.error_vel = np.zeros(3)
        self.error_R = np.zeros(3)
        self.max_thrust = 0.720 
        self.angle_limit = 55.0
        self.prev_euler = np.zeros(3)
        self.t0 = 0
        self.first = True

    def set_trajectory(self, start, end, duration, order=7):
        """
        Set the trajectory for the quadrotor.

        Args:
            start (list): Initial conditions for the trajectory [x(0), y(0), z(0), roll(0), pitch(0), yaw(0)].
            end (list): Final conditions for the trajectory [x(T), y(T), z(T), roll(T), pitch(T), yaw(T)].
            duration (float): Duration of the trajectory.
            order (int): Polynomial order for the trajectory (default is 7 for minimum snap).
        """
        self.traj_gen = TrajectoryGenerator(start, end, duration, order)

    def compute_position_error(self, pos, t):
        """
        Compute position and velocity errors.

        Args:
            pos (ndarray): Current position of the quadrotor.
            t (float): Current time in the simulation.
        """
        self.rt = self.traj_gen.evaluate(t, order=0)  # Desired position
        self.vt = self.traj_gen.evaluate(t, order=1)  # Desired velocity
        self.at = self.traj_gen.evaluate(t, order=2)  # Desired acceleration
        vel = (pos - self.prev_pos) / (t - self.t ) if self.t > 0 else np.zeros(3)  # Current velocity
        # print(f" t {t}")
        # print(f" pos {pos}")
        # print(f" rt {self.rt[:3]}")
        # print(f" vt {self.vt[:3]}")
        # print(f" vel {vel}")
        self.error_pos = pos - self.rt[:3]
        self.error_vel = vel - self.vt[:3]
        self.prev_pos = pos
        

    def compute_desired_force(self):
        """
        Compute the desired force vector based on position and velocity errors.
        """
        gravity_force = np.array([0, 0, self.m * 9.81])
        # print(f" 1 {-self.Kp @ self.error_pos}")
        # print(f" 2 {-self.Kv @ self.error_vel}")
        # print(f" pos error {self.error_pos}")
        # print(f" vel error {self.error_vel}")
        # print(f" gravity {gravity_force}")
        # print(f" ma {self.m * self.at[:3]}")
        error_magnitude = np.linalg.norm(self.error_pos)
        Kp_curr = self.Kp * self.Kp_scale_factor * error_magnitude
        self.Fdes = (-self.Kp @ self.error_pos  - self.Kv @ self.error_vel  + gravity_force + self.m * self.at[:3]) 

    def compute_desired_thrust(self, zB):
        """
        Compute the total thrust (u1) in the body frame.

        Args:
            zB (ndarray): z-axis of the body frame.
        """
        self.u1 = np.dot(self.Fdes, zB) * 0.12  ################## Added 0.15 to scale the thrust

    def compute_desired_orientation(self):
        """
        Compute the desired orientation matrix based on the desired force.
        """
        zB = self.Fdes / np.linalg.norm(self.Fdes)
        xC = np.array([np.cos(self.rt[5]), np.sin(self.rt[5]), 0])
        yB = np.cross(zB, xC) / np.linalg.norm(np.cross(zB, xC))
        xB = np.cross(yB, zB)
        self.Rdes = np.vstack((xB, yB, zB)).T

    def compute_angular_velocity(self, current_euler, dt):
        """
        Compute the angular velocity based on current and previous Euler angles.

        Args:
            current_euler (ndarray): Current Euler angles [roll, pitch, yaw] in radians.
            dt (float): Time step.

        Returns:
            ndarray: Angular velocity vector [ωx, ωy, ωz].
        """
        prev_euler = self.prev_euler  # Previous Euler angles

        # Compute the angular velocity in Euler angles
        delta_euler = current_euler - prev_euler

        # Ensure continuity for yaw (handle wrapping between -π and π)
        delta_euler[2] = (delta_euler[2] + np.pi) % (2 * np.pi) - np.pi

        # Angular velocity in roll, pitch, yaw rates
        angular_velocity_euler = delta_euler / dt

        # Save the current Euler angles for the next time step
        self.prev_euler = current_euler

        return angular_velocity_euler


    def compute_orientation_error(self, RB):
        """
        Compute orientation error in the body frame.

        Args:
            RB (ndarray): Current rotation matrix of the quadrotor.
        """
        er = 0.5 * (np.dot(self.Rdes.T, RB) - np.dot(RB.T, self.Rdes))
        self.error_R = np.array([er[2, 1], er[0, 2], er[1, 0]])

    def euler_to_rotation_matrix(self,roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a 3x3 rotation matrix.

        Args:
            roll (float): Rotation about the x-axis in radians.
            pitch (float): Rotation about the y-axis in radians.
            yaw (float): Rotation about the z-axis in radians.

        Returns:
            ndarray: 3x3 rotation matrix.
        """
        # Rotation around x-axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Rotation around y-axis (pitch)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        # Rotation around z-axis (yaw)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Combined rotation matrix (order: Rz * Ry * Rx)
        R = R_z @ R_y @ R_x
        return R

    def _convert_to_pwm(self, control_input, max_value):
        """
        Convert a control input to a PWM signal.

        Args:
            control_input (float): Control input value.
            max_value (float): Maximum value for the control input.

        Returns:
            int: PWM value in the range [1000, 2000].
        """
        control_input = np.clip(control_input, -max_value, max_value)
        pwm = int(((control_input + max_value) / (2 * max_value)) * 1000) + 1000
        return pwm
    
    def step(self, pos,current_euler, t):
        """
        Execute one step of the control algorithm.

        Args:
            pos (ndarray): Current position of the quadrotor.
            RB (ndarray): Current rotation matrix of the quadrotor.
            current_quat (ndarray): Current quaternion of the quadrotor.
            t (float): Current time.

        Returns:
            ndarray: Control inputs [u1, u2, u3, u4].
        """
        if self.first:
            self.t0 = t
            self.first = False
            
        if self.t > self.traj_gen.duration:
            return 1000, 1500, 1500, 1500, True, [0.0,0.0,0.0,0.0,0.0,0.0]  
        
        # ## Update the trajectory if error is more than 0.4
        # if np.linalg.norm(self.error_pos) > 1.4:
        #     # self.traj_gen.duration = self.traj_gen.duration - self.t
        #     self.traj_gen.start = [pos[0], pos[1], pos[2], current_euler[0], current_euler[1], current_euler[2]]
        #     print(f" traj_gen.start {self.traj_gen.start}")
        #     self.traj_gen = TrajectoryGenerator(self.traj_gen.start, self.traj_gen.end, self.traj_gen.duration, 7)
        #     self.t0 = t
        
        dt = t - self.t - self.t0 if self.t > 0 else 1e-3  # Prevent division by zero
        self.compute_position_error(pos, t-self.t0)
        
        self.compute_desired_force()
        RB = self.euler_to_rotation_matrix(current_euler[0], current_euler[1], current_euler[2])
        self.compute_desired_thrust(RB[:, 2])
        # print(self.Fdes)    
        # print(RB[:,2])
        # print(self.u1)
        self.compute_desired_orientation()
        
        
        self.compute_orientation_error(RB)
        
        omega_actual = self.compute_angular_velocity(current_euler, dt)
        
        omega_desired = self.traj_gen.evaluate(t, order=1)[3:]
        
        error_omega = omega_actual - omega_desired
        
        u = -self.Kr * self.error_R - self.Kw * error_omega
        # print(f" u {self.u1}")
        self.u1 = np.clip(self.u1, 0, self.max_thrust)
        
        # Convert control inputs to PWM signals
        pwm_thrust = int((self.u1 / self.max_thrust) * 1000) + 1000
        pwm_roll = self._convert_to_pwm(np.rad2deg( u[0]), self.angle_limit)
        pwm_pitch = self._convert_to_pwm(np.rad2deg( u[1]), self.angle_limit)
        pwm_yaw = self._convert_to_pwm(np.rad2deg(u[2]), self.angle_limit)
        # print(f" pwm_thrust {pwm_thrust}")
        self.t = t - self.t0
        
        # Return PWM values
        return pwm_thrust, pwm_roll, pwm_pitch, pwm_yaw, False, self.rt

