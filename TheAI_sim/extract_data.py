#!/usr/bin/env python3
import os
import csv
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from rosidl_runtime_py.utilities import get_message
import argparse

# Import the Actuator message type
from interfaces.msg import Actuator

def process_bag(bag_path, output_dir, max_speed=0.5, max_turn_rate=30.0):
    """Process a bag file and extract data to CSV files"""
    # Get bag name without extension for file naming
    bag_name = os.path.basename(os.path.dirname(bag_path))
    
    # Set up bag reader
    storage_options = StorageOptions(
        uri=os.path.dirname(bag_path),
        storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    # Initialize bag reader
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Data dictionaries for each topic type
    imu_data = {}
    odom_data = {}
    actuator_data = {}
    
    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topic_types}
    
    print(f"Processing bag: {bag_path}")
    
    # First pass: read all messages and store in memory
    while reader.has_next():
        topic_name, data, t_stamp = reader.read_next()
        
        # Convert timestamp to seconds
        if hasattr(t_stamp, 'sec') and hasattr(t_stamp, 'nanosec'):
            timestamp = t_stamp.sec + (t_stamp.nanosec / 1e9)
        elif isinstance(t_stamp, int):
            # If t_stamp is already an integer (nanoseconds), convert to seconds
            timestamp = t_stamp / 1e9
        else:
            # If we can't determine the timestamp format, skip
            print(f"Warning: Unknown timestamp format: {type(t_stamp)} for topic {topic_name}")
            continue
        
        if topic_name == '/sookshma_00/imu/data':
            msg = deserialize_message(data, Imu)
            imu_data[timestamp] = {
                'linear_acc_x': msg.linear_acceleration.x,
                'linear_acc_y': msg.linear_acceleration.y,
                'linear_acc_z': msg.linear_acceleration.z,
                'angular_vel_x': msg.angular_velocity.x,
                'angular_vel_y': msg.angular_velocity.y,
                'angular_vel_z': msg.angular_velocity.z
            }
            
        elif topic_name == '/sookshma_00/odometry_sim':
            msg = deserialize_message(data, Odometry)
            odom_data[timestamp] = {
                'position_x': msg.pose.pose.position.x,
                'position_y': msg.pose.pose.position.y,
                'position_z': msg.pose.pose.position.z,
                'orientation_x': msg.pose.pose.orientation.x,
                'orientation_y': msg.pose.pose.orientation.y,
                'orientation_z': msg.pose.pose.orientation.z,
                'orientation_w': msg.pose.pose.orientation.w,
                'linear_vel_x': msg.twist.twist.linear.x,
                'linear_vel_y': msg.twist.twist.linear.y,
                'linear_vel_z': msg.twist.twist.linear.z,
                'angular_vel_x': msg.twist.twist.angular.x,
                'angular_vel_y': msg.twist.twist.angular.y,
                'angular_vel_z': msg.twist.twist.angular.z
            }
            
        elif topic_name == '/sookshma_00/actuator_cmd':
            msg = deserialize_message(data, Actuator)
            
            # Default values
            rudder = 0.0
            propeller = 0.0
            
            # Extract actuator values based on names
            for i, name in enumerate(msg.actuator_names):
                if name == 'cs_1' and i < len(msg.actuator_values):
                    rudder = msg.actuator_values[i]
                elif name == 'propeller' and i < len(msg.actuator_values):
                    propeller = msg.actuator_values[i]
            
            actuator_data[timestamp] = {
                'rudder': rudder,
                'propeller': propeller
            }
            
    # Print statistics about collected data
    print(f"Collected data:")
    print(f"  IMU: {len(imu_data)} samples (100Hz)")
    print(f"  Odometry: {len(odom_data)} samples (10Hz)")
    print(f"  Actuator: {len(actuator_data)} samples (10Hz)")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Create combined dataset centered on actuator timestamps
    combined_data = create_combined_dataset(actuator_data, odom_data, imu_data)
    combined_path = os.path.join(output_dir, f"{bag_name}_combined.csv")
    save_dict_to_csv(combined_data, combined_path, 'timestamp')
    
    print(f"\nProcessed bag: {bag_path}")
    print(f"Data saved to {output_dir}/{bag_name}_*.csv")
    
    # Return statistics
    return {
        'bag': bag_name,
        'imu_samples': len(imu_data),
        'odom_samples': len(odom_data),
        'actuator_samples': len(actuator_data),
        'combined_samples': len(combined_data)
    }

def save_dict_to_csv(data_dict, filepath, time_key='timestamp'):
    """Save a dictionary of dictionaries to a CSV file"""
    if not data_dict:
        print(f"Warning: No data to save to {filepath}")
        return
    
    # Get all possible keys from the data
    keys = set()
    for timestamp, values in data_dict.items():
        keys.update(values.keys())
    
    # Sort keys for consistent output, but ensure timestamp is first
    sorted_keys = sorted(list(keys))
    
    # Write data to CSV
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        # Write header with timestamp as the first column
        header = [time_key] + sorted_keys
        writer.writerow(header)
        
        # Write data rows
        for timestamp, values in sorted(data_dict.items()):
            row = [timestamp]  # Start with timestamp
            for key in sorted_keys:  # Add other values
                row.append(values.get(key, ''))
            writer.writerow(row)
    
    print(f"Saved {len(data_dict)} samples to {filepath}")

def find_closest_timestamp(target_time, time_dict, method='closest'):
    """
    Find the closest timestamp in time_dict to target_time
    
    Args:
        target_time: Target timestamp
        time_dict: Dictionary with timestamps as keys
        method: Method to use ('closest', 'before', 'after')
    
    Returns:
        Closest timestamp, or None if not found
    """
    if not time_dict:
        return None
    
    if method == 'closest':
        closest_time = min(time_dict.keys(), key=lambda x: abs(x - target_time))
    elif method == 'before':
        # Find the most recent timestamp before target_time
        before_times = [t for t in time_dict.keys() if t <= target_time]
        if not before_times:
            return None
        closest_time = max(before_times)
    elif method == 'after':
        # Find the earliest timestamp after target_time
        after_times = [t for t in time_dict.keys() if t >= target_time]
        if not after_times:
            return None
        closest_time = min(after_times)
    else:
        raise ValueError(f"Invalid method: {method}")
    
    # Print a warning if the time difference is large
    time_diff = abs(closest_time - target_time)
    if time_diff > 0.1:  # More than 100ms difference
        print(f"Warning: Large time difference ({time_diff:.3f}s) when syncing data using method '{method}'")
    
    return closest_time

def create_combined_dataset(actuator_data, odom_data, imu_data):
    """Create a combined dataset by syncing all data sources based on actuator timestamps"""
    combined_data = {}
    
    # Stats for reporting
    total_time_diff_imu = 0
    total_time_diff_odom = 0
    max_time_diff_imu = 0
    max_time_diff_odom = 0
    
    # Collect unique IMU samples used to verify we're not repeating data
    used_imu_timestamps = set()
    
    # Use actuator timestamps as the primary time reference
    for act_timestamp in sorted(actuator_data.keys()):
        # Find closest odometry data - can use closest method since odom is at same rate as actuator
        odom_timestamp = find_closest_timestamp(act_timestamp, odom_data, method='closest')
        if odom_timestamp is None:
            continue
        
        # Find most recent IMU data before this timestamp (IMU is at 100Hz)
        imu_timestamp = find_closest_timestamp(act_timestamp, imu_data, method='before')
        if imu_timestamp is None:
            # If no IMU data before this timestamp, try getting the earliest after
            imu_timestamp = find_closest_timestamp(act_timestamp, imu_data, method='after')
            if imu_timestamp is None:
                continue
        
        # Calculate time differences
        odom_diff = abs(act_timestamp - odom_timestamp)
        imu_diff = abs(act_timestamp - imu_timestamp)
        
        # Track statistics
        total_time_diff_odom += odom_diff
        total_time_diff_imu += imu_diff
        max_time_diff_odom = max(max_time_diff_odom, odom_diff)
        max_time_diff_imu = max(max_time_diff_imu, imu_diff)
        
        # Track used IMU timestamps
        used_imu_timestamps.add(imu_timestamp)
        
        # Combine data
        combined_entry = {}
        combined_entry.update(actuator_data[act_timestamp])
        combined_entry.update(odom_data[odom_timestamp])
        combined_entry.update(imu_data[imu_timestamp])
        
        # Store in combined dictionary
        combined_data[act_timestamp] = combined_entry
    
    # Print statistics
    count = len(combined_data)
    unique_imu_ratio = len(used_imu_timestamps) / count if count > 0 else 0
    
    if count > 0:
        avg_time_diff_odom = total_time_diff_odom / count
        avg_time_diff_imu = total_time_diff_imu / count
        print(f"Created combined dataset with {count} samples")
        print(f"Average time difference for odometry sync: {avg_time_diff_odom:.6f}s (max: {max_time_diff_odom:.6f}s)")
        print(f"Average time difference for IMU sync: {avg_time_diff_imu:.6f}s (max: {max_time_diff_imu:.6f}s)")
        print(f"Used {len(used_imu_timestamps)} unique IMU samples for {count} data points (ratio: {unique_imu_ratio:.2f})")
        
        if unique_imu_ratio < 0.8:
            print(f"WARNING: Low unique IMU sample ratio ({unique_imu_ratio:.2f}) indicates potential data quality issues!")
    else:
        print("Warning: No combined data points created!")
    
    return combined_data

def process_all_bags(base_dir, output_dir, max_speed=0.5, max_turn_rate=30.0):
    """Process all bag files in subdirectories of the given directory"""
    # Initialize ROS context
    rclpy.init()
    
    # Find all bag files
    bag_files = []
    for root, dirs, files in os.walk(base_dir):
        if any(f.endswith('.db3') for f in files):
            db3_file = next(f for f in files if f.endswith('.db3'))
            bag_files.append(os.path.join(root, db3_file))
    
    # Process each bag file
    stats = []
    for bag_file in bag_files:
        try:
            stat = process_bag(bag_file, output_dir, max_speed=max_speed, max_turn_rate=max_turn_rate)
            stats.append(stat)
        except Exception as e:
            print(f"Error processing {bag_file}: {str(e)}")
    
    # Print statistics
    print("\nData Extraction Statistics:")
    print("=" * 100)
    print(f"{'Bag Name':<20} | {'IMU':<8} | {'Odom':<8} | {'Actuator':<10} | {'Combined':<10}")
    print("-" * 100)
    
    for stat in stats:
        print(f"{stat['bag']:<20} | {stat['imu_samples']:<8} | {stat['odom_samples']:<8} | {stat['actuator_samples']:<10} | {stat['combined_samples']:<10}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Extract data from ROS2 bags')
    parser.add_argument('--base_dir', type=str, default='../data_from_test', 
                        help='Base directory containing ROS2 bags')
    parser.add_argument('--output_dir', type=str, default=None, 
                        help='Output directory for extracted data')
    parser.add_argument('--max_speed', type=float, default=0.5, 
                        help='Maximum allowed vessel speed in m/s')
    parser.add_argument('--max_turn_rate', type=float, default=30.0,
                        help='Maximum allowed turn rate in degrees/s')
    args = parser.parse_args()
    
    # Define base directory for bag files and output directory for CSV files
    base_dir = args.base_dir
    
    # Set default output directory if not provided
    if args.output_dir is None:
        output_dir = os.path.join(os.getcwd(), 'extracted_data')
    else:
        output_dir = args.output_dir
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Process all bags
    process_all_bags(base_dir, output_dir, max_speed=args.max_speed, max_turn_rate=args.max_turn_rate) 