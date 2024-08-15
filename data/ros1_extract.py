import rosbag
import json
import rospy
from std_msgs.msg import String  # Import the appropriate message types
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

def extract_data_from_ros1_bag(ros1_bag_path, topics, output_file):
    extracted_data = {topic: [] for topic in topics}
    
    # Open the ROS1 bag
    with rosbag.Bag(ros1_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            # Convert the message to a dictionary (modify as per your message type)
            msg_dict = message_to_dict(msg)
            extracted_data[topic].append({'time': t.to_sec(), 'data': msg_dict})
    
    # Save the extracted data to a JSON file
    with open(output_file, 'w') as f:
        json.dump(extracted_data, f, indent=4)

def message_to_dict(msg):
    # This function converts a ROS1 message to a dictionary
    # Modify this function to handle the specific message types you're extracting

    # Handle String messages
    if msg._type == "std_msgs/String":
        return {'data': msg.data}
    
    # Handle Imu messages
    if msg._type == "sensor_msgs/Imu":
        return {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            },
            'orientation_covariance': msg.orientation_covariance,
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'angular_velocity_covariance': msg.angular_velocity_covariance,
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
            'linear_acceleration_covariance': msg.linear_acceleration_covariance
        }

    # Handle NavSatFix messages
    if msg._type == "sensor_msgs/NavSatFix":
        return {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': msg.position_covariance,
            'position_covariance_type': msg.position_covariance_type,
            'status': {
                'status': msg.status.status,
                'service': msg.status.service
            }
        }

    return {}

if __name__ == "__main__":
    ros1_bag_path = 'bagfiles/free_running_1.bag'
    output_file = 'bagfiles/free_running_1_extracted_data.json'
    topics_to_extract = ['/ardusimple/gnss', '/sbg/imu/data', '/absoluteEncoder']

    extract_data_from_ros1_bag(ros1_bag_path, topics_to_extract, output_file)
