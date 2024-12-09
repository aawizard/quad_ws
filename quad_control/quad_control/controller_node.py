import rclpy
from rclpy.node import Node
from quad_interfaces.msg import QuadCmd
from controllers.pid_altitude import PID_alttitude, PID_roll_pitch
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import numpy as np

class Controller_pid(Node):

    def __init__(self):
        super().__init__('controller_pid')
        
        # Declare parameters for simulation and desired position
        self.declare_parameter("use_drone", True)
        self.declare_parameter("use_sim", False)
        self.declare_parameter("desired_x", -0.05)
        self.declare_parameter("desired_y", 0.0)
        self.declare_parameter("desired_z", 0.5)
        
        
        # Fetch parameter values
        self.use_drone = self.get_parameter("use_drone").value
        self.use_sim = self.get_parameter("use_sim").value
        self.desired_position = np.array([
            self.get_parameter("desired_x").value,
            self.get_parameter("desired_y").value,
            self.get_parameter("desired_z").value
        ])
        
        # Initialize publisher, subscriber, and timers
        self.publisher_ = self.create_publisher(QuadCmd, 'quad_ctrl', 10)
        self.desired_pose_pub = self.create_publisher(PoseStamped, "desired_pose", 10)
        self.actual_path_pub = self.create_publisher(Path, 'actual_path', 10)
        self.actual_path = Path()
        timer_period = 0.01  # seconds
        self.subscription = self.create_subscription(PoseStamped, 'quad_pose', self.listener_callback, 10)
        
        # Set PID gains based on whether simulation or real drone is used
        if self.use_sim:
            self.pid_altitude = PID_alttitude(kp=0.3, ki=0.05, kd=0.09, dt=timer_period, feedforward= 0.3)
            self.pid_x = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
            self.pid_y = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
        else:
            self.pid_altitude = PID_alttitude(kp=0.3, ki=0.05, kd=0.1, dt=timer_period, feedforward= 0.45, max_thrust= 0.8)
            self.pid_x = PID_roll_pitch(kp=0.1,  ki=0.002, kd=0.38, dt=timer_period)
            self.pid_y = PID_roll_pitch(kp=0.1, ki=0.002, kd=0.38, dt=timer_period)
        
        
        # Initialize quadcopter command message
        self.quad_cmd = QuadCmd()
        self.quad_cmd.roll = 1500
        self.quad_cmd.pitch = 1500
        self.quad_cmd.yaw = 1500
        self.quad_cmd.throttle = 900
        self.quad_cmd.armed = False
        self.flag = 0
        
        # Set up timer for callback execution
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize current position
        self.curr_position = np.array([0.0, 0.0, 0.0])
        
        #set desiredpose
        self.desired_pose = PoseStamped()
        self.desired_pose.pose.position.x = self.desired_position[0]
        self.desired_pose.pose.position.y = self.desired_position[1]
        self.desired_pose.pose.position.z = self.desired_position[2]
        
    def set_desired_position(self, position):
        self.desired_position = position
        

    def listener_callback(self, msg):
        # Update current position from subscribed message
        self.curr_position[0] = msg.pose.position.x
        self.curr_position[1] = msg.pose.position.y
        self.curr_position[2] = msg.pose.position.z
        # self.desired_pose.pose.position.z = msg.pose.position.z
        # self.desired_pose.pose.position.x = msg.pose.position.x
        # self.desired_pose.pose.position.y = msg.pose.position.y
        self.actual_path.poses.append(msg)

    def timer_callback(self):
        # Check if it's time to arm the drone
        if 500 < self.flag < 700:
            self.quad_cmd.armed = True
        elif self.flag >= 700:
            # Compute errors
            error = self.desired_position - self.curr_position
            thrust = self.pid_altitude.step(error[2], self.curr_position[2])
            pitch = self.pid_x.step( error[0])
            roll = self.pid_y.step(-1* error[1])
            self.get_logger().info(f"error z: {error[2]}, error y: {error[1]}, error x: {error[0]}")
            
            # Update quad command with PID outputs
            self.quad_cmd.throttle = thrust
            self.quad_cmd.roll = roll   
            self.quad_cmd.pitch = pitch
            self.quad_cmd.armed = True
        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.actual_path.header.frame_id = 'map'
        self.actual_path_pub.publish(self.actual_path)
        
        self.desired_pose.header.frame_id = "map"
        self.desired_pose.header.stamp = self.get_clock().now().to_msg()

        # Publish command and increment flag
        self.publisher_.publish(self.quad_cmd)
        self.desired_pose_pub.publish(self.desired_pose)
        self.flag += 1

def main(args=None):
    rclpy.init(args=args)

    # Instantiate and spin the controller_pid
    controller_pid = Controller_pid()
    rclpy.spin(controller_pid)

    # Clean up
    controller_pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
