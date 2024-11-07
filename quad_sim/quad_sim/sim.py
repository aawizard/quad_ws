import rclpy
from rclpy.node import Node
from quad_interfaces.msg import QuadCmd
from std_msgs.msg import String
from dynamics.quadEnv import quadrotor
import math
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class QuadSim(Node):

    def __init__(self):
        super().__init__('quad_sim')
        self.subscription = self.create_subscription(
            QuadCmd, '/quad_ctrl',
            self.ctrl_callback,
            10)
        timer_period = 0.01
        self.subscription  # prevent unused variable warning
        self.qd = quadrotor(Ts = 1.0/16.0, USE_PWM = 1, USE_PID = 0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(PoseStamped, '/sim/quad_pose', 10)
        self.ctrl = QuadCmd()
        self.qd.des_xyz(1.0, 2.0, 3.0)
        self.qd.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pose = PoseStamped()
        self.tf_broadcaster = TransformBroadcaster(self)

    def tf_pose(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose.pose.position.x
        t.transform.translation.y = self.pose.pose.position.y
        t.transform.translation.z = self.pose.pose.position.z
        t.transform.rotation.x = self.pose.pose.orientation.x
        t.transform.rotation.y = self.pose.pose.orientation.y
        t.transform.rotation.z = self.pose.pose.orientation.z
        t.transform.rotation.w = self.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)
        
    def euler_to_quaternion(self,roll, pitch, yaw):
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w).
        """

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    def ctrl_callback(self, msg):
        self.ctrl = msg
        
    def timer_callback(self):
        if self.ctrl.armed:
            self.get_logger().info("Arming")
            self.qd.input_vector[0] = self.ctrl.throttle
            self.qd.input_vector[1] = self.ctrl.roll 
            self.qd.input_vector[2] = self.ctrl.pitch
            self.qd.input_vector[3] = self.ctrl.yaw
            # self.qd.PID_position() 
            state = self.qd.step(self.qd.state, self.qd.input_vector)
            # print(state[0], state[1], state[2], state[9], state[10], state[11])
            x,y,z,w = self.euler_to_quaternion(state[0], state[1], state[2])
            self.pose.pose.position.x = state[9]
            self.pose.pose.position.y = state[10]
            self.pose.pose.position.z = state[11]
            self.pose.pose.orientation.x = x
            self.pose.pose.orientation.y = y
            self.pose.pose.orientation.z = z
            self.pose.pose.orientation.w = w
            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.header.frame_id = 'map'
            self.publisher_.publish(self.pose)
            self.tf_pose()
            
            
            


def main(args=None):
    rclpy.init(args=args)

    quad_sim = QuadSim()

    rclpy.spin(quad_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quad_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()