import rclpy
from rclpy.node import Node
from quad_interfaces.msg import QuadCmd
from controllers.traj_controller import TrajectoryController
from controllers.pid_altitude import PID_alttitude, PID_roll_pitch
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from enum import Enum, auto


class State(Enum):
    """
    Declaring diffrent states the drone can be in.

    Four possible states
    """    
    DISARM = auto()
    INITIAL = auto()
    START_POINT = auto()
    MIN_SNAP = auto()
    STABALIZE = auto()
    

class Controller_min(Node):

    def __init__(self):
        super().__init__('controller_min')
        
        # Declare parameters for simulation and desired position
        self.declare_parameter("use_drone", True)
        self.declare_parameter("use_sim", False)
        self.declare_parameter("desired_x", 0.4)
        self.declare_parameter("desired_y", 0.4)
        self.declare_parameter("desired_z", 0.8)
        
        
        # Fetch parameter values
        self.use_drone = self.get_parameter("use_drone").value
        self.use_sim = self.get_parameter("use_sim").value
        self.desired_position = np.array([
            self.get_parameter("desired_x").value,
            self.get_parameter("desired_y").value,
            self.get_parameter("desired_z").value
        ])
        self.first_position = False
        self.state = State.INITIAL
        self.start_position = np.array([0.0, 0.0, 1.9])
        # Initialize publisher, subscriber, and timers
        self.publisher_ = self.create_publisher(QuadCmd, 'quad_ctrl', 10)
        self.traj_path_pub = self.create_publisher(Path, 'traj_path', 10)
        self.actual_path_pub = self.create_publisher(Path, 'actual_path', 10)
        self.desired_pose_pub = self.create_publisher(PoseStamped, "desired_pose", 10)
        self.traj_path = Path()
        self.actual_path = Path()
        timer_period = 0.01  # seconds
        self.subscription = self.create_subscription(PoseStamped, 'quad_pose', self.listener_callback, 10)
        self.traj_controller_duration = 4.0
        
        # Initialize current position
        self.curr_position = np.array([0.0, 0.0, 0.0])
        self.curr_euler = np.array([0.0, 0.0, 0.0])
        
        # Set PID gains based on whether simulation or real drone is used
        if self.use_sim:
            self.controller = TrajectoryController(m=0.11,
                                                   Kp=np.diag([1.3, 1.3, 14.5]),
                                                   Kv=np.diag([1.0, 1.0, 1.7]),
                                                   Kr=2.9,
                                                   Kw=0.9)
            
            self.pid_altitude = PID_alttitude(kp=0.3, ki=0.05, kd=0.09, dt=timer_period, feedforward= 0.3)
            self.pid_x = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
            self.pid_y = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
        else:
            self.controller = TrajectoryController(m=0.11,
                                                   Kp=np.diag([1.3, 1.3, 11.5]),
                                                   Kv=np.diag([1.0, 1.0, 1.7]),
                                                   Kr=2.9,
                                                   Kw=0.9)
            
            self.pid_altitude = PID_alttitude(kp=0.3, ki=0.05, kd=0.09, dt=timer_period, feedforward=0.45)
            self.pid_x = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
            self.pid_y = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
        #     self.pid_altitude = PID_alttitude(kp=1.4, ki=0.2, kd=0.05, dt=timer_period)
        #     self.pid_x = PID_roll_pitch(kp=0.8, ki=0.01, kd=0.3, dt=timer_period)
        #     self.pid_y = PID_roll_pitch(kp=0.8, ki=0.0, kd=0.3, dt=timer_period)
        
        # self.pid_yaw = PID_roll_pitch(kp=0.3, ki=0.01, kd=0.4, dt=timer_period)
        
        # Initialize quadcopter command message
        self.quad_cmd = QuadCmd()
        self.quad_cmd.roll = 1500
        self.quad_cmd.pitch = 1500
        self.quad_cmd.yaw = 1500
        self.quad_cmd.throttle = 900
        self.quad_cmd.armed = False
        self.flag = 0
        self.done = False
        
        # Set up timer for callback execution
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #set desiredpose
        self.desired_pose = PoseStamped()
        self.desired_pose.pose.position.x = self.desired_position[0]
        self.desired_pose.pose.position.y = self.desired_position[1]
        self.desired_pose.pose.position.z = self.desired_position[2]
        
    
        
        
    def euler_to_quaternion(self,roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
     
    def quaternion_to_euler(self,q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).

        Args:
            q (ndarray): Quaternion [q0, q1, q2, q3], where
                        q0 is the scalar part and (q1, q2, q3) are the vector parts.

        Returns:
            tuple: Euler angles (roll, pitch, yaw) in radians.
        """
        q0, q1, q2, q3 = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1**2 + q2**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
 
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2**2 + q3**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])    
    
    def add_trajectory_point(self, rt):
        if rt is None or rt[:2] == [0.0, 0.0, 0.0]:
            pass
        # Create a new pose message
        pose = PoseStamped()
        pose.pose.position.x = rt[0]
        pose.pose.position.y = rt[1]
        pose.pose.position.z = rt[2]
        orientation = self.euler_to_quaternion(rt[3], rt[4], rt[5])
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        self.traj_path.poses.append(pose)
    
    def set_desired_position(self, position):
        self.desired_position = position

    def listener_callback(self, msg):
        
        # Update current position from subscribed message
        self.curr_position[0] = msg.pose.position.x
        self.curr_position[1] = msg.pose.position.y
        self.curr_position[2] = msg.pose.position.z
        self.curr_euler = self.quaternion_to_euler([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        self.actual_path.poses.append(msg)
        if not self.first_position:
            self.first_position = True
            self.controller.set_trajectory(start=np.concatenate([self.curr_position, np.array([0.0, 0.0, 0.0])]),
                                            end=np.concatenate([self.desired_position, np.array([0.0, 0.0, 0.0])]),
                                            duration=self.traj_controller_duration)
            

    def timer_callback(self):
        # Check if it's time to arm the drone
        if self.state == State.DISARM:
            self.quad_cmd.armed = False
            self.quad_cmd.throttle = 1000
            if self.flag > 500:
                self.state = State.INITIAL
        elif self.state == State.INITIAL:
            self.quad_cmd.armed = True
            if self.flag > 700:
                self.state = State.MIN_SNAP
        
        # elif self.state == State.START_POINT:
        #     error = self.start_position - self.curr_position
        #     thrust = self.pid_altitude.step(error[2], self.curr_position[2])
        #     pitch = self.pid_x.step( error[0])
        #     roll = self.pid_y.step(-1* error[1])
        #     self.get_logger().info("PID control")
        #     # self.get_logger().info(f"error z: {error[2]}, error y: {error[1]}, error x: {error[0]}")
        #     self.quad_cmd.throttle = int(thrust)
        #     # self.quad_cmd.roll = 1500
        #     # self.quad_cmd.pitch = 1500
        #     self.quad_cmd.roll = int(roll)   
        #     self.quad_cmd.pitch = int(pitch)
        #     self.quad_cmd.armed = True
            
        #     if abs(np.linalg.norm(error)) < 0.1:
        #         self.state = State.MIN_SNAP
        #         self.flag = 0
        #         self.controller.set_trajectory(start=np.concatenate([self.curr_position, np.array([0.0, 0.0, 0.0])]),
        #                                     end=np.concatenate([self.desired_position, np.array([0.0, 0.0, 0.0])]),
        #                                     duration=self.traj_controller_duration)
            
        elif self.state == State.MIN_SNAP:
            self.time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
            thrust, roll, pitch, yaw, self.done, rt = self.controller.step(self.curr_position, self.curr_euler, self.time)
            self.add_trajectory_point(rt)
            # self.get_logger().info("Traj control")
            # self.get_logger().info(f"thrust: {thrust}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
            # Update quad command with traj controller outputs
            self.quad_cmd.throttle = int(thrust)
            self.quad_cmd.roll = int(roll)   
            self.quad_cmd.pitch = int(pitch)
            self.quad_cmd.armed = True
            error = self.desired_position - self.curr_position
            if abs(np.linalg.norm(error)) < 0.1:
                self.state = State.STABALIZE
                
        elif self.state == State.STABALIZE:
            self.quad_cmd.armed = False
            
            # Compute errors
            # error = self.desired_position - self.curr_position
            # thrust = self.pid_altitude.step(error[2], self.curr_position[2])
            # pitch = self.pid_x.step( error[0])
            # roll = self.pid_y.step(-1* error[1])
            # self.get_logger().info("PID control")
            # # self.get_logger().info(f"error z: {error[2]}, error y: {error[1]}, error x: {error[0]}")
            # self.quad_cmd.throttle = int(thrust)
            # # self.quad_cmd.roll = 1500
            # # self.quad_cmd.pitch = 1500
            # self.quad_cmd.roll = int(roll)   
            # self.quad_cmd.pitch = int(pitch)
            # self.quad_cmd.armed = True
            
            
        # Publish command and increment flag
        self.publisher_.publish(self.quad_cmd)
        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.actual_path.header.frame_id = "map"
        self.traj_path.header.stamp = self.get_clock().now().to_msg()
        self.traj_path.header.frame_id = "map"
        self.desired_pose.header.frame_id = "map"
        self.desired_pose.header.stamp = self.get_clock().now().to_msg()
        
        self.traj_path_pub.publish(self.traj_path)
        self.actual_path_pub.publish(self.actual_path)
        self.desired_pose_pub.publish(self.desired_pose)
        self.flag += 1

def main(args=None):
    rclpy.init(args=args)

    # Instantiate and spin the controller_min
    controller_min = Controller_min()
    rclpy.spin(controller_min)

    # Clean up
    controller_min.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
