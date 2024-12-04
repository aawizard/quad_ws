import rclpy
from rclpy.node import Node
from quad_interfaces.msg import QuadCmd
from std_msgs.msg import String
from dynamics.quadEnv import quadrotor
import math
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class QuadSim(Node):

    def __init__(self):
        super().__init__('quad_sim')
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('frame_id', 'base_link')
        
        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_z = self.get_parameter('z').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            QuadCmd, 'quad_ctrl',
            self.ctrl_callback,
            10)
        
        timer_period = 0.01
        self.subscription  # prevent unused variable warning
        self.qd = quadrotor(Ts=1.0/16.0, USE_PWM=1, USE_PID=0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(PoseStamped, 'quad_pose', 10)
        self.ctrl = QuadCmd()
        self.qd.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, self.initial_x, self.initial_y, self.initial_z]
        self.pose = PoseStamped()
        # self.pose.pose.position.x = self.initial_x
        # self.pose.pose.position.y = self.initial_y
        # self.pose.pose.position.z = self.initial_z
        
        marker_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_marker = self.create_publisher(
            MarkerArray, "visualization_marker_array", marker_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.make_drone_makerer()

    def make_drone_part(self, id, scale, pose, type, color):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = id
        m.type = type
        m.action = Marker.ADD
        m.scale.x = scale[0]
        m.scale.y = scale[1]
        m.scale.z = scale[2]
        m.pose.position.x = pose[0]
        m.pose.position.y = pose[1]
        m.pose.position.z = pose[2]
        m.pose.orientation.x = pose[3]
        m.pose.orientation.y = pose[4]
        m.pose.orientation.z = pose[5]
        m.pose.orientation.w = pose[6]
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]
        m.frame_locked = True
        return m

    def make_drone_makerer(self):
        self.drone = MarkerArray()
        self.drone.markers.append(
            self.make_drone_part(
                1, [0.4, 0.02, 0.02], [0.0, 0.0, 0.0, 0.0, 0.0, 0.3826834324, 0.9238795325], Marker.CUBE, [1.0, 1.0, 0.0, 1.0]
            )
        )
        self.drone.markers.append(
            self.make_drone_part(
                2, [0.4, 0.02, 0.02], [0.0, 0.0, 0.0, 0.0, 0.0, -0.3826834324, 0.9238795325], Marker.CUBE, [1.0, 1.0, 0.0, 1.0]
            )
        )
        self.drone.markers.append(
            self.make_drone_part(
                3, [0.1, 0.06, 0.02], [0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0], Marker.CYLINDER, [1.0, 0.0, 0.0, 1.0]
            )
        )
        self.drone.markers.append(
            self.make_drone_part(
                4, [0.1, 0.06, 0.02], [0.15, -0.15, 0.0, 0.0, 0.0, 0.0, 1.0], Marker.CYLINDER, [1.0, 0.0, 0.0, 1.0]
            )
        )
        self.drone.markers.append(
            self.make_drone_part(
                5, [0.1, 0.06, 0.02], [-0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0], Marker.CYLINDER, [0.0, 0.0, 1.0, 1.0]
            )
        )
        self.drone.markers.append(
            self.make_drone_part(
                6, [0.1, 0.06, 0.02], [-0.15, -0.15, 0.0, 0.0, 0.0, 0.0, 1.0], Marker.CYLINDER, [0.0, 0.0, 1.0, 1.0]
            )
        )
        self.pub_marker.publish(self.drone)

    def tf_pose(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.pose.pose.position.x
        t.transform.translation.y = self.pose.pose.position.y
        t.transform.translation.z = self.pose.pose.position.z
        t.transform.rotation.x = self.pose.pose.orientation.x
        t.transform.rotation.y = self.pose.pose.orientation.y
        t.transform.rotation.z = self.pose.pose.orientation.z
        t.transform.rotation.w = self.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
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
            self.qd.input_vector[0] = self.ctrl.throttle
            self.qd.input_vector[1] = self.ctrl.roll 
            self.qd.input_vector[2] = self.ctrl.pitch
            self.qd.input_vector[3] = self.ctrl.yaw
            state = self.qd.step(self.qd.state, self.qd.input_vector)
            x, y, z, w = self.euler_to_quaternion(state[0], state[1], state[2])
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

    # Update initial position and frame ID as needed
    quad_sim = QuadSim()

    rclpy.spin(quad_sim)

    quad_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
