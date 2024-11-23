from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
from module_kinematics import eul_to_quat


def extract_data_from_bag(bag_path):
    # Open the bag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Create dictionaries for storing data
    imu_data = {}
    gps_data = {}
    odom_data = {}

    # Message type mapping
    message_types = {
        'sensor_msgs/msg/Imu': Imu,
        'sensor_msgs/msg/NavSatFix': NavSatFix,
        'nav_msgs/msg/Odometry': Odometry
    }

    # Read topics and types from the bag
    topic_types = reader.get_all_topics_and_types()
    topic_name_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topic_names = []

    for topic in topic_types:
        topic_name = topic.name
        topic_type = topic.type
        msg_class = message_types.get(topic_type, None)
        if not msg_class:
            continue
        
        topic_names.append(topic_name)

        # Initialize topic-specific data structures
        if topic_type == 'sensor_msgs/msg/Imu':
            imu_data[topic_name] = {
                "time": [],
                "ang_vel": [],
                "acc": [],
                "quat": [],
                "eul": []
            }
        elif topic_type == 'sensor_msgs/msg/NavSatFix':
            gps_data[topic_name] = {
                "time": [],
                "lla": []
            }
        elif topic_type == 'nav_msgs/msg/Odometry':
            odom_data[topic_name] = {
                "time": [],
                "pos": [],
                "quat": [],
                "eul": [],
                "lin_vel": [],
                "ang_vel": []
            }

    # Set up filter for the specific topic
    storage_filter = StorageFilter(topics=topic_names)
    reader.set_filter(storage_filter)

    # Extract messages for the filtered topic
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()        
        topic_type = topic_name_types[topic_name]
        msg_class = message_types.get(topic_type, None)
        if not msg_class:
            continue
        
        msg = deserialize_message(data, msg_class)
        
        if topic_type == 'sensor_msgs/msg/Imu':
            imu_data[topic_name]["time"].append(timestamp / 1e9)
            imu_data[topic_name]["ang_vel"].append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            imu_data[topic_name]["acc"].append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            imu_data[topic_name]["quat"].append(quat)
            eul = eul_to_quat(quat, deg=True)  # Convert to Euler angles
            imu_data[topic_name]["eul"].append(eul)
        elif topic_type == 'sensor_msgs/msg/NavSatFix':
            gps_data[topic_name]["time"].append(timestamp / 1e9)
            gps_data[topic_name]["lla"].append([msg.latitude, msg.longitude, msg.altitude])
        elif topic_type == 'nav_msgs/msg/Odometry':            
            odom_data[topic_name]["time"].append(timestamp / 1e9)
            odom_data[topic_name]["pos"].append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            odom_data[topic_name]["quat"].append(quat)
            eul = eul_to_quat(quat, deg=True)  # Convert to Euler angles
            odom_data[topic_name]["eul"].append(eul)
            odom_data[topic_name]["lin_vel"].append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            odom_data[topic_name]["ang_vel"].append([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

    # Convert lists to numpy arrays
    for data_dict in [imu_data, gps_data, odom_data]:
        for topic, data in data_dict.items():
            for key in data:
                data[key] = np.array(data[key])

    return imu_data, gps_data, odom_data


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: python extract_bag_data.py <bag_path>")
        sys.exit(1)

    bag_path = sys.argv[1]
    imu_data, gps_data, odom_data = extract_data_from_bag(bag_path)

    print("IMU Data:")
    for topic, data in imu_data.items():
        print(f"Topic: {topic}")
        for key, value in data.items():
            print(f"  {key}: {value.shape}")

    print("\nGPS Data:")
    for topic, data in gps_data.items():
        print(f"Topic: {topic}")
        for key, value in data.items():
            print(f"  {key}: {value.shape}")

    print("\nOdometry Data:")
    for topic, data in odom_data.items():
        print(f"Topic: {topic}")
        for key, value in data.items():
            print(f"  {key}: {value.shape}")
