import os
import numpy as np
import rosbag2_py
from scipy.io import savemat
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, NavSatFix
from builtin_interfaces.msg import Time
import rclpy

def read_ros2_bag(bag_path):
    # Initialize ROS 2
    rclpy.init()

    # Create a reader to access the bag
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get list of topics
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    # print(type_map)

    # Prepare data structures to store the results
    data = {
        'Imu': {
            'time': [],
            'orientation': [],
            'angular_velocity': [],
            'linear_acceleration': [],
        },
        'NavSatFix': {
            'time': [],
            'latitude': [],
            'longitude': [],
            'altitude': [],
        },
        'Actuators':{
            'time': [],
            'rudder': [],
            'propeller': [],
        }
    }

    # Read messages from the bag
    while reader.has_next():
        (topic, data_serialized, t) = reader.read_next()
        msg_type = type_map[topic]

        # Deserialize the message based on its type
        if msg_type == 'sensor_msgs/msg/Imu':
            imu_msg = deserialize_message(data_serialized, Imu)
            timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            data['Imu']['time'].append(timestamp)
            data['Imu']['orientation'].append([
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w
            ])
            data['Imu']['angular_velocity'].append([
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            ])
            data['Imu']['linear_acceleration'].append([
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            ])

        elif msg_type == 'sensor_msgs/msg/NavSatFix':
            navsat_msg = deserialize_message(data_serialized, NavSatFix)
            timestamp = navsat_msg.header.stamp.sec + navsat_msg.header.stamp.nanosec * 1e-9
            data['NavSatFix']['time'].append(timestamp)
            data['NavSatFix']['latitude'].append(navsat_msg.latitude)
            data['NavSatFix']['longitude'].append(navsat_msg.longitude)
            data['NavSatFix']['altitude'].append(navsat_msg.altitude)
            
        # elif msg_type == 'interfaces/msg/Actuator':
        #     actuator_msg = deserialize_message(data_serialized, Ac)
        #     timestamp = navsat_msg.header.stamp.sec + navsat_msg.header.stamp.nanosec * 1e-9
        #     data['NavSatFix']['time'].append(timestamp)
        #     data['NavSatFix']['latitude'].append(navsat_msg.latitude)
        #     data['NavSatFix']['longitude'].append(navsat_msg.longitude)
        #     data['NavSatFix']['altitude'].append(navsat_msg.altitude)

    # Convert lists to numpy arrays
    for key in data['Imu']:
        data['Imu'][key] = np.array(data['Imu'][key])

    for key in data['NavSatFix']:
        data['NavSatFix'][key] = np.array(data['NavSatFix'][key])

    # Save the data as a .mat file
    mat_filename = os.path.join(bag_path, 'ros2_data.mat')
    # savemat(mat_filename, data)

    # print(f"Data saved to {mat_filename}")

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':

    bag_file_path = os.path.join("..", "2024_10_12", "run_02")  # Replace with your ROS 2 bag file path
    read_ros2_bag(bag_file_path)
