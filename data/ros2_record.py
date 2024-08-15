import json
import rclpy
from rclpy.time import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import String  # Import the appropriate message types
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from interfaces.msg import Actuator
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import numpy as np

def record_data_to_ros2_bag(json_file, ros2_bag_path):
    # Load the extracted data from the JSON file
    with open(json_file, 'r') as f:
        extracted_data = json.load(f)

    # Initialize ROS2 and setup the bag writer
    rclpy.init()
    storage_options = StorageOptions(uri=ros2_bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # Register topics before writing any messages
    for topic in extracted_data.keys():
        create_topic(writer, topic)

    # Iterate over the extracted data and write to the ROS2 bag
    for topic, messages in extracted_data.items():
        for message in messages:
            # Create the ROS2 message (modify based on message types)
            ros2_msg, topic_remap = dict_to_message(topic, message['data'], message['time'])
            serialized_msg = serialize_message(ros2_msg)
            writer.write(topic_remap, serialized_msg, int(message['time'] * 1e9))

    # ROS2 will automatically close the writer when it goes out of scope
    rclpy.shutdown()

def create_topic(writer, topic):
    # Define the topic metadata (type and reliability)
    if topic == '/absoluteEncoder':
        topic_metadata = TopicMetadata(name='/makara_00/encoders', type='interfaces/msg/Actuator', serialization_format='cdr')
    elif topic == '/sbg/imu/data':
        topic_metadata = TopicMetadata(name='/makara_00/imu', type='sensor_msgs/msg/Imu', serialization_format='cdr')
    elif topic == '/ardusimple/gnss':
        topic_metadata = TopicMetadata(name='/makara_00/gps', type='sensor_msgs/msg/NavSatFix', serialization_format='cdr')
    else:
        raise ValueError(f"Unknown topic: {topic}")
    
    # Register the topic with the writer
    writer.create_topic(topic_metadata)

def dict_to_message(topic, msg_dict, timestamp):

    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)
    ros_time = Time(seconds=sec, nanoseconds=nanosec).to_msg()

    # Handle String messages
    if topic == '/absoluteEncoder':
        encoders_rms = np.array([1e-1 * np.pi / 180, 1e-1 / 60])
        encoders_cov = np.diag(encoders_rms ** 2)
        
        msg = Actuator()
        msg.header.stamp = ros_time
        msg.rudder = np.float64(msg_dict['data'].split(',')[0])
        msg.propeller = np.float64(msg_dict['data'].split(',')[1])
        msg.covariance = encoders_cov.flatten()
        topic_remap = '/makara_00/encoders'
        return msg, topic_remap
    
    # Handle Imu messages
    if topic == '/sbg/imu/data':
        msg = Imu()
        msg.header.stamp = ros_time
        msg.orientation.x = msg_dict['orientation']['x']
        msg.orientation.y = msg_dict['orientation']['y']
        msg.orientation.z = msg_dict['orientation']['z']
        msg.orientation.w = msg_dict['orientation']['w']
        msg.orientation_covariance = msg_dict['orientation_covariance']
        msg.angular_velocity.x = msg_dict['angular_velocity']['x']
        msg.angular_velocity.y = msg_dict['angular_velocity']['y']
        msg.angular_velocity.z = msg_dict['angular_velocity']['z']
        msg.angular_velocity_covariance = msg_dict['angular_velocity_covariance']
        msg.linear_acceleration.x = msg_dict['linear_acceleration']['x']
        msg.linear_acceleration.y = msg_dict['linear_acceleration']['y']
        msg.linear_acceleration.z = msg_dict['linear_acceleration']['z']
        msg.linear_acceleration_covariance = msg_dict['linear_acceleration_covariance']
        topic_remap = '/makara_00/imu'
        return msg, topic_remap

    # Handle NavSatFix messages
    if topic == '/ardusimple/gnss':
        msg = NavSatFix()
        msg.header.stamp = ros_time
        msg.latitude = msg_dict['latitude']
        msg.longitude = msg_dict['longitude']
        msg.altitude = msg_dict['altitude']
        msg.position_covariance = msg_dict['position_covariance']
        msg.position_covariance_type = msg_dict['position_covariance_type']
        msg.status.status = msg_dict['status']['status']
        msg.status.service = msg_dict['status']['service']
        topic_remap = '/makara_00/gps'
        return msg, topic_remap

    return None, None

if __name__ == "__main__":
    json_file = 'free_running_1_extracted_data.json'
    ros2_bag_path = 'free_running_1_ros2'

    record_data_to_ros2_bag(json_file, ros2_bag_path)
