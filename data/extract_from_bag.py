import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_data_from_ros2_bag(bag_file_path, topic_name):
    # Initialize the storage and reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')  # Use default options

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get the list of topics available in the bag
    topics_and_types = reader.get_all_topics_and_types()
    
    # Find the type of the topic
    topic_type = None
    for topic in topics_and_types:
        if topic.name == topic_name:
            topic_type = topic.type
            break
    
    if topic_type is None:
        raise ValueError(f"Topic '{topic_name}' not found in the bag file.")

    # Get the message class for the topic type
    message_class = get_message(topic_type)
    
    # List to store the extracted data
    data_list = []

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == topic_name:
            # Deserialize the message
            message = deserialize_message(data, message_class)
            data_list.append(message)  # Store the deserialized message

    return data_list

if __name__ == "__main__":
    # Replace with the path to your ROS2 bag file and the topic you want to extract
    bag_file_path = "bagfiles/2024_08_13/bagfile_wo_ekf_0.db3"
    topic_name = "/imu/data"

    rclpy.init()  # Initialize ROS2
    extracted_data = extract_data_from_ros2_bag(bag_file_path, topic_name)
    rclpy.shutdown()  # Shutdown ROS2
    
    # Print the extracted data (or do something else with it)
    for data in extracted_data:
        print(data)

