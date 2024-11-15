import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import 3D plotting capabilities

def plot_3d_drone_position(bag_file_path):
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

    # Check if necessary topics are in the bag
    if "/quad_pose" not in type_dict:
        print("Topic '/quad_pose' not found in the bag file.")
        return

    # Get message type for /quad_pose
    quad_pose_type = get_message(type_dict["/quad_pose"])

    # Initialize data lists for 3D plot
    x_values, y_values, z_values = [], [], []

    # Read messages from bag file
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == "/quad_pose":
            msg = deserialize_message(data, quad_pose_type)
            x_values.append(msg.pose.position.x)
            y_values.append(msg.pose.position.y)
            z_values.append(msg.pose.position.z)

    # Plotting the 3D position
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_values, y_values, z_values, label='Drone Position Path', color='blue')
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.set_title("3D Position of Drone")
    ax.grid(True)
    ax.legend()

    plt.show()

    # Shutdown ROS
    rclpy.shutdown()

# Example usage
plot_3d_drone_position("rosbag/test2")
