import rclpy
from rclpy.node import Node
import csv
from quad_interfaces.msg import QuadCmd
from geometry_msgs.msg import PoseStamped


class Recorded(Node):

    def __init__(self):
        super().__init__('recorder')
        
        # Initialize subscriptions
        self.subscription = self.create_subscription(
            QuadCmd,
            'quad_ctrl',
            self.quad_ctrl_callback,
            10)
        self.sub_dronepose = self.create_subscription(
            PoseStamped,
            '/quad_pose',
            self.quad_pose_callback,
            10)
        self.sub_simpose = self.create_subscription(
            PoseStamped,
            '/sim/quad_pose',
            self.sim_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Open CSV file and write headers
        self.csv_file = open('recorded_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write header
        self.csv_writer.writerow(['Timestamp',"Timestamp_nsec" 'Message Type', 'Throttle', 'Quad Pose Z', 'Sim Pose Z'])

    def quad_ctrl_callback(self, msg: QuadCmd):
        # Log and write only throttle value to CSV
        self.get_logger().info(f'I heard QuadCmd throttle: {msg.throttle}')
        self.csv_writer.writerow([
            self.get_clock().now().to_msg().sec,  # Timestamp
            self.get_clock().now().to_msg().nanosec,
            'QuadCmd',  # Message Type
            msg.throttle,  # Throttle
            '', ''  # Empty fields for quad_pose.z and sim_pose.z
        ])

    def quad_pose_callback(self, msg: PoseStamped):
        # Log and write only position.z for quad_pose to CSV
        self.get_logger().info(f'I heard quad_pose position.z: {msg.pose.position.z}')
        self.csv_writer.writerow([
            self.get_clock().now().to_msg().sec,
            self.get_clock().now().to_msg().nanosec,# Timestamp
            'quad_pose',  # Message Type
            '',  # Empty field for throttle
            msg.pose.position.z,  # Quad Pose Z
            ''  # Empty field for sim_pose.z
        ])

    def sim_pose_callback(self, msg: PoseStamped):
        # Log and write only position.z for sim_pose to CSV
        self.get_logger().info(f'I heard sim_pose position.z: {msg.pose.position.z}')
        self.csv_writer.writerow([
            self.get_clock().now().to_msg().sec,
            self.get_clock().now().to_msg().nanosec,# Timestamp
            'sim_pose',  # Message Type
            '', '',  # Empty fields for throttle and quad_pose.z
            msg.pose.position.z  # Sim Pose Z
        ])

    def __del__(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Recorded()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the node is properly destroyed
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
