import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R  # For quaternion to euler conversion

def plot_drone_yaw(bag_file_path):
    # Initialize ROS and create a node
    rclpy.init()

    # Set up ROS bag reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get all topics and types from the bag file
    topic_types = reader.get_all_topics_and_types()
    type_dict = {topic.name: topic.type for topic in topic_types}

    # Check if the /quad_pose topic is in the bag
    if "/quad_pose" not in type_dict:
        print("Topic '/quad_pose' not found in the bag file.")
        return

    # Get message type for /quad_pose
    quad_pose_type = get_message(type_dict["/quad_pose"])

    # Initialize data lists for yaw plot
    times, yaw_values = [], []

    # Read messages from bag file
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        time_in_seconds = timestamp * 1e-9  # Convert timestamp to seconds

        if topic == "/quad_pose":
            msg = deserialize_message(data, quad_pose_type)
            # Extract quaternion
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            
            # Convert quaternion to Euler angles
            rotation = R.from_quat([qx, qy, qz, qw])
            roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)  # Get yaw in degrees

            # Append data to lists
            times.append(time_in_seconds)
            yaw_values.append(roll)

    # Plotting the yaw over time
    plt.figure(figsize=(10, 6))
    plt.plot(times, yaw_values, label='Yaw', color='purple')
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (degrees)")
    plt.title("Drone Yaw over Time")
    plt.grid(True)
    plt.legend()
    plt.show()

    # Shutdown ROS
    rclpy.shutdown()

# Example usage
plot_drone_yaw("rosbag/test3")
