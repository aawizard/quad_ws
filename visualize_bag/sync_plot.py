# import rclpy
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message
# import rosbag2_py
# import matplotlib.pyplot as plt
# import numpy as np

# def plot_rosbag_data(bag_file_path):
#     # Initialize ROS and create a node
#     rclpy.init()

#     # Set up ROS bag reader
#     storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='mcap')
#     converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#     reader = rosbag2_py.SequentialReader()
#     reader.open(storage_options, converter_options)

#     # Get all topics and types from the bag file
#     topic_types = reader.get_all_topics_and_types()
#     type_dict = {topic.name: topic.type for topic in topic_types}

#     # Check if necessary topics are in the bag
#     required_topics = ["/quad_ctrl", "/sim/quad_pose", "/quad_pose"]
#     for topic in required_topics:
#         if topic not in type_dict:
#             print(f"Topic '{topic}' not found in the bag file.")
#             return

#     # Get message types for each topic
#     quad_ctrl_type = get_message(type_dict["/quad_ctrl"])
#     sim_pose_type = get_message(type_dict["/sim/quad_pose"])
#     quad_pose_type = get_message(type_dict["/quad_pose"])

#     # Initialize data lists
#     times_quad_pose, z_values_quad_pose = [], []
#     times_sim_pose, z_values_sim_pose = [], []
#     times_quad_ctrl, throttle_values_quad_ctrl = [], []

#     # Read messages from bag file
#     while reader.has_next():
#         topic, data, timestamp = reader.read_next()
#         time_in_seconds = timestamp * 1e-9  # Convert timestamp to seconds

#         if topic == "/quad_ctrl":
#             msg = deserialize_message(data, quad_ctrl_type)
#             times_quad_ctrl.append(time_in_seconds)
#             throttle_values_quad_ctrl.append(msg.throttle)
        
#         elif topic == "/sim/quad_pose":
#             msg = deserialize_message(data, sim_pose_type)
#             times_sim_pose.append(time_in_seconds)
#             z_values_sim_pose.append(msg.pose.position.z)
        
#         elif topic == "/quad_pose":
#             msg = deserialize_message(data, quad_pose_type)
#             times_quad_pose.append(time_in_seconds)
#             z_values_quad_pose.append(msg.pose.position.z)

#     # Determine common time range for x-axis limits
#     start_time = max(min(times_quad_pose), min(times_sim_pose), min(times_quad_ctrl))
#     end_time = min(max(times_quad_pose), max(times_sim_pose), max(times_quad_ctrl))

#     # Plotting
#     plt.figure(figsize=(10, 8))

#     # Top plot: Quad Pose Z Position vs Time
#     plt.subplot(3, 1, 1)
#     plt.plot(times_quad_pose, z_values_quad_pose, label='Quad Pose Z', color='blue')
#     plt.xlim(start_time, end_time)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Z Position (m)")
#     plt.title("Quad Pose Z Position vs Time")
#     plt.grid(True)

#     # Middle plot: Sim Pose Z Position vs Time
#     plt.subplot(3, 1, 2)
#     plt.plot(times_sim_pose, z_values_sim_pose, label='Sim Pose Z', color='green')
#     plt.xlim(start_time, end_time)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Z Position (m)")
#     plt.title("Sim Pose Z Position vs Time")
#     plt.grid(True)

#     # Bottom plot: QuadCtrl Throttle vs Time
#     plt.subplot(3, 1, 3)
#     plt.plot(times_quad_ctrl, throttle_values_quad_ctrl, label='QuadCtrl Throttle', color='red')
#     plt.xlim(start_time, end_time)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Throttle")
#     plt.title("QuadCtrl Throttle vs Time")
#     plt.grid(True)

#     plt.tight_layout()
#     plt.show()

#     # Shutdown ROS
#     rclpy.shutdown()

# # Example usage
# plot_rosbag_data("rosbag/subset2")

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np

def plot_rosbag_data(bag_file_path, axis):
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
    required_topics = ["/quad_ctrl", "/quad_pose"]
    for topic in required_topics:
        if topic not in type_dict:
            print(f"Topic '{topic}' not found in the bag file.")
            return

    # Get message types for each topic
    quad_ctrl_type = get_message(type_dict["/quad_ctrl"])
    quad_pose_type = get_message(type_dict["/quad_pose"])

    # Initialize data lists
    times_quad_pose, z_values_quad_pose = [], []
    times_quad_ctrl, throttle_values_quad_ctrl = [], []

    # Read messages from bag file
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        time_in_seconds = timestamp * 1e-9  # Convert timestamp to seconds

        if topic == "/quad_ctrl":
            msg = deserialize_message(data, quad_ctrl_type)
            times_quad_ctrl.append(time_in_seconds)
            if axis =="z":
                throttle_values_quad_ctrl.append(msg.throttle)
            elif axis =="y":
                throttle_values_quad_ctrl.append(msg.roll)
            elif axis =="x":
                throttle_values_quad_ctrl.append(msg.pitch)
        
        elif topic == "/quad_pose":
            msg = deserialize_message(data, quad_pose_type)
            times_quad_pose.append(time_in_seconds)
            if axis =="z":
                z_values_quad_pose.append(msg.pose.position.z)
            elif axis =="y":
                z_values_quad_pose.append(msg.pose.position.y)
            elif axis =="x":
                z_values_quad_pose.append(msg.pose.position.x)

    # Determine common time range for x-axis limits
    start_time = max(min(times_quad_pose), min(times_quad_ctrl))
    end_time = min(max(times_quad_pose), max(times_quad_ctrl))

    # Plotting
    plt.figure(figsize=(10, 6))

    # Top plot: Quad Pose Z Position vs Time
    plt.subplot(2, 1, 1)
    plt.plot(times_quad_pose, z_values_quad_pose, label=f'Quad Pose {axis}', color='blue')
    plt.xlim(start_time, end_time)
    plt.xlabel("Time (s)")
    plt.ylabel(f"{axis} Position (m)")
    plt.title(f"Quad Pose {axis} Position vs Time")
    plt.grid(True)

    # Bottom plot: QuadCtrl Throttle vs Time
    plt.subplot(2, 1, 2)
    if axis =="z":
        plt.plot(times_quad_ctrl, throttle_values_quad_ctrl, label='QuadCtrl Throttle', color='red')
        plt.ylabel("Throttle")
        plt.title("QuadCtrl Throttle vs Time")
    elif axis =="y":
        plt.plot(times_quad_ctrl, throttle_values_quad_ctrl, label='QuadCtrl Roll', color='red')
        plt.ylabel("Roll")
        plt.title("QuadCtrl Roll vs Time")
    elif axis =="x":
        plt.plot(times_quad_ctrl, throttle_values_quad_ctrl, label='QuadCtrl Pitch', color='red')
        plt.ylabel("Pitch")
        plt.title("QuadCtrl Pitch vs Time")
    plt.xlim(start_time, end_time)
    plt.xlabel("Time (s)")
    plt.grid(True)
    # plt.title(f"{bag_file_path} {axis}")

    plt.tight_layout()
    plt.savefig(f"{bag_file_path}/{axis}.png")
    plt.show()
    

    # Shutdown ROS
    rclpy.shutdown()

# Example usage
# plot_rosbag_data("rosbag/test1", "x")
# plot_rosbag_data("rosbag/test1", "y")
# plot_rosbag_data("rosbag/test1", "z")

# plot_rosbag_data("rosbag/test2", "x")
# plot_rosbag_data("rosbag/test2", "y")
# plot_rosbag_data("rosbag/test2", "z")


# plot_rosbag_data("rosbag/test3", "x")
# plot_rosbag_data("rosbag/test3", "y")
# plot_rosbag_data("rosbag/test3", "z")

# plot_rosbag_data("rosbag/test4", "x")
# plot_rosbag_data("rosbag/test4", "y")
# plot_rosbag_data("rosbag/test4", "z")



# plot_rosbag_data("rosbag/test5", "x")
# plot_rosbag_data("rosbag/test5", "y")
# plot_rosbag_data("rosbag/test5", "z")

plot_rosbag_data("rosbag/test11", "x")
plot_rosbag_data("rosbag/test11", "y")
plot_rosbag_data("rosbag/test11", "z")
