import rclpy
from rclpy.node import Node
from quad_interfaces.msg import QuadCmd
from controllers.pid_altitude import PID_alttitude, PID_roll_pitch
from geometry_msgs.msg import PoseStamped
import numpy as np

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.declare_parameter("use_drone", True)
        self.declare_parameter("use_sim", False)
        self.use_drone = self.get_parameter("use_drone").value
        self.use_sim = self.get_parameter("use_sim").value
        
        self.publisher_ = self.create_publisher(QuadCmd, '/quad_ctrl', 10)
        timer_period = 0.01  # seconds
        if self.use_sim:
            self.subscription = self.create_subscription(PoseStamped, '/sim/quad_pose', self.listener_callback, 10)
            self.pid_altitude = PID_alttitude(kp=1.7, ki=0.31, kd=0.2, dt=timer_period)
            self.pid_x = PID_roll_pitch(kp=0.3, ki=0.01, kd=0.4, dt=timer_period)
            self.pid_y = PID_roll_pitch(kp=0.3, ki=0.0, kd=0.4, dt=timer_period)
        else:
            self.subscription = self.create_subscription(PoseStamped, '/quad_pose', self.listener_callback, 10)
            self.pid_altitude = PID_alttitude(kp=1.2, ki=0.05, kd=1.2, dt=timer_period)
            self.pid_x = PID_roll_pitch(kp=0.3, ki=0.00, kd=0.2, dt=timer_period)
            self.pid_y = PID_roll_pitch(kp=0.3, ki=0.0, kd=0.2, dt=timer_period)
            
        self.pid_yaw = PID_roll_pitch(kp=0.3, ki=0.01, kd=0.4, dt=timer_period)
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        
        self.desired_position = np.array([0.2, 0.3, 0.3])
        self.curr_position =  np.array([0.0, 0.0, 0.0])
        self.quad_cmd = QuadCmd()
        self.quad_cmd.roll = 1500
        self.quad_cmd.pitch = 1500
        self.quad_cmd.yaw = 1500
        self.quad_cmd.throttle = 900
        self.quad_cmd.armed = False
        self.flag = 0
        self.arm = False
        
    def set_desired_altitude(self, position):
        self.desired_position = position
        
    def listener_callback(self, msg):
        self.curr_position[0] = msg.pose.position.x
        self.curr_position[1] = msg.pose.position.y
        self.curr_position[2] = msg.pose.position.z
        

    def timer_callback(self):
        if self.flag > 500 and self.flag < 700:
            self.quad_cmd.armed = True
        elif self.flag > 700:
        
        
            error = self.desired_position - self.curr_position
            thrust = self.pid_altitude.step(error[2])
            roll = self.pid_x.step(-1 * error[1])
            pitch = self.pid_y.step(error[0])
            self.get_logger().info(f"error z: {error[2]}")
            
            self.quad_cmd.throttle = thrust
            self.quad_cmd.roll = roll   
            self.quad_cmd.pitch = pitch
            self.quad_cmd.armed = True
        self.publisher_.publish(self.quad_cmd)
        self.flag+=1

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()