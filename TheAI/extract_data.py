#!/usr/bin/env python3
import os
import csv
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from rosidl_runtime_py.utilities import get_message
import sys
import glob
from datetime import datetime
from scipy.signal import savgol_filter
from scipy.ndimage import gaussian_filter1d
import argparse

# Import the Actuator message type
from interfaces.msg import Actuator

def init_csv_files(output_dir, output_prefix):
    """Initialize CSV files with headers for each data type"""
    os.makedirs(output_dir, exist_ok=True)
    
    # IMU data CSV
    imu_csv_path = os.path.join(output_dir, f"{output_prefix}_imu.csv")
    with open(imu_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 
            'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z'
        ])
    
    # Odometry data CSV
    odom_csv_path = os.path.join(output_dir, f"{output_prefix}_odom.csv")
    with open(odom_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z'
        ])
    
    # Smoothed odometry data CSV
    smoothed_odom_csv_path = os.path.join(output_dir, f"{output_prefix}_smoothed_odom.csv")
    with open(smoothed_odom_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z'
        ])
    
    # Actuator data CSV
    actuator_csv_path = os.path.join(output_dir, f"{output_prefix}_actuator.csv")
    with open(actuator_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'rudder', 'propeller'
        ])
    
    # Combined data (for ML) CSV
    combined_csv_path = os.path.join(output_dir, f"{output_prefix}_combined.csv")
    with open(combined_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'rudder', 'propeller',
            'position_x', 'position_y'
        ])
    
    # Smoothed combined data (for ML) CSV
    smoothed_combined_csv_path = os.path.join(output_dir, f"{output_prefix}_smoothed_combined.csv")
    with open(smoothed_combined_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'rudder', 'propeller',
            'position_x', 'position_y'
        ])
    
    return {
        'imu': imu_csv_path,
        'odom': odom_csv_path,
        'smoothed_odom': smoothed_odom_csv_path,
        'actuator': actuator_csv_path,
        'combined': combined_csv_path,
        'smoothed_combined': smoothed_combined_csv_path
    }

def smooth_position_data(odom_data, min_distance=0.3, window_size=15):
    """
    Smooths position data and filters out points that are too close together
    
    Args:
        odom_data (dict): Dictionary containing odometry data
        min_distance (float): Minimum distance between consecutive points (in meters)
        window_size (int): Window size for smoothing
    
    Returns:
        dict: Dictionary containing smoothed and filtered odometry data
    """
    if len(odom_data) < window_size:
        print("Warning: Not enough data points for smoothing. Using original data.")
        return odom_data
    
    # Extract timestamps and position data
    timestamps = sorted(odom_data.keys())
    positions_x = np.array([odom_data[t]['position_x'] for t in timestamps])
    positions_y = np.array([odom_data[t]['position_y'] for t in timestamps])
    
    # Apply Gaussian filter for smoothing
    # Adjust sigma based on the noise level - higher sigma = more smoothing
    positions_x_smoothed = gaussian_filter1d(positions_x, sigma=2.0)
    positions_y_smoothed = gaussian_filter1d(positions_y, sigma=2.0)
    
    # Create a new dictionary for smoothed data
    smoothed_odom_data = {}
    
    # Filter points to ensure minimum distance and add back to dictionary
    last_x, last_y = positions_x_smoothed[0], positions_y_smoothed[0]
    last_timestamp = timestamps[0]
    
    # Always include the first point
    smoothed_odom_data[last_timestamp] = dict(odom_data[last_timestamp])
    smoothed_odom_data[last_timestamp]['position_x'] = positions_x_smoothed[0]
    smoothed_odom_data[last_timestamp]['position_y'] = positions_y_smoothed[0]
    
    for i in range(1, len(timestamps)):
        current_x, current_y = positions_x_smoothed[i], positions_y_smoothed[i]
        current_timestamp = timestamps[i]
        
        # Calculate distance from last included point
        distance = np.sqrt((current_x - last_x)**2 + (current_y - last_y)**2)
        
        # Only include points that are at least min_distance meters away from the last included point
        if distance >= min_distance:
            smoothed_odom_data[current_timestamp] = dict(odom_data[current_timestamp])
            smoothed_odom_data[current_timestamp]['position_x'] = current_x
            smoothed_odom_data[current_timestamp]['position_y'] = current_y
            
            # Update last included point
            last_x, last_y = current_x, current_y
            last_timestamp = current_timestamp
    
    print(f"Original data points: {len(odom_data)}, Smoothed and filtered data points: {len(smoothed_odom_data)}")
    return smoothed_odom_data

def process_bag(bag_path, output_dir, min_distance=0.3):
    """Process a bag file and extract data to CSV files"""
    # Get bag name without extension for file naming
    bag_name = os.path.basename(os.path.dirname(bag_path))
    
    # Initialize CSV files
    csv_files = init_csv_files(output_dir, bag_name)
    
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
    
    # Dictionaries to store the latest data from each topic
    latest_imu = None
    latest_odom = None
    latest_actuator = None
    
    # Create CSV writers
    imu_file = open(csv_files['imu'], 'a', newline='')
    imu_writer = csv.writer(imu_file)
    
    odom_file = open(csv_files['odom'], 'a', newline='')
    odom_writer = csv.writer(odom_file)
    
    actuator_file = open(csv_files['actuator'], 'a', newline='')
    actuator_writer = csv.writer(actuator_file)
    
    combined_file = open(csv_files['combined'], 'a', newline='')
    combined_writer = csv.writer(combined_file)
    
    # Variable to track the last recorded actuator timestamp
    last_actuator_timestamp = None
    
    # Data dictionaries for synchronization
    imu_data = {}
    odom_data = {}
    actuator_data = {}
    
    # Get topic type
    topic_types = reader.get_all_topics_and_types()
    
    # Create a mapping of topic names to message types
    topic_type_map = {topic.name: topic.type for topic in topic_types}
    
    print(f"Processing bag: {bag_path}")
    
    # Process all messages
    while reader.has_next():
        topic_name, data, t_stamp = reader.read_next()
        
        # Convert timestamp to seconds
        # Handle both integer timestamps and object timestamps with sec/nanosec attributes
        if hasattr(t_stamp, 'sec') and hasattr(t_stamp, 'nanosec'):
            timestamp = t_stamp.sec + (t_stamp.nanosec / 1e9)
        elif isinstance(t_stamp, int):
            # If t_stamp is already an integer (nanoseconds), convert to seconds
            timestamp = t_stamp / 1e9
        else:
            # If we can't determine the timestamp format, use a counter as fallback
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
            
            # Write to IMU CSV
            imu_writer.writerow([
                timestamp,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            ])
            
        elif topic_name == '/sookshma_00/odometry':
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
            
            # Write to Odometry CSV
            odom_writer.writerow([
                timestamp,
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z
            ])
            
        elif topic_name == '/sookshma_00/actuator_cmd':
            msg = deserialize_message(data, Actuator)
            
            # Default values
            rudder = 0.0
            propeller = 0.0
            
            # Extract actuator values based on names
            for i, name in enumerate(msg.actuator_names):
                if name == 'rudder' and i < len(msg.actuator_values):
                    rudder = msg.actuator_values[i]
                elif name == 'propeller' and i < len(msg.actuator_values):
                    propeller = msg.actuator_values[i]
            
            actuator_data[timestamp] = {
                'rudder': rudder,
                'propeller': propeller
            }
            
            # Write to Actuator CSV
            actuator_writer.writerow([
                timestamp,
                rudder, propeller
            ])
            
            last_actuator_timestamp = timestamp
    
    # Close initial files
    imu_file.close()
    odom_file.close()
    actuator_file.close()
    
    # Smooth the position data
    smoothed_odom_data = smooth_position_data(odom_data, min_distance=min_distance)
    
    # Write smoothed odometry data to CSV
    smoothed_odom_file = open(csv_files['smoothed_odom'], 'a', newline='')
    smoothed_odom_writer = csv.writer(smoothed_odom_file)
    
    for timestamp, data in smoothed_odom_data.items():
        smoothed_odom_writer.writerow([
            timestamp,
            data['position_x'], data['position_y'], data['position_z'],
            data['orientation_x'], data['orientation_y'], data['orientation_z'], data['orientation_w'],
            data['linear_vel_x'], data['linear_vel_y'], data['linear_vel_z'],
            data['angular_vel_x'], data['angular_vel_y'], data['angular_vel_z']
        ])
    
    smoothed_odom_file.close()
    
    # Create combined dataset with synchronized timestamps
    # We'll only include data points when actuator commands are available
    combined_data = []
    
    # For each actuator timestamp, find the closest IMU and odometry data
    for act_timestamp, act_values in actuator_data.items():
        # Find closest IMU timestamp
        closest_imu_timestamp = min(imu_data.keys(), key=lambda x: abs(x - act_timestamp), default=None)
        closest_odom_timestamp = min(odom_data.keys(), key=lambda x: abs(x - act_timestamp), default=None)
        
        if closest_imu_timestamp and closest_odom_timestamp:
            # Get the corresponding data
            imu = imu_data[closest_imu_timestamp]
            odom = odom_data[closest_odom_timestamp]
            
            # Write to combined CSV file
            combined_writer.writerow([
                act_timestamp,
                imu['linear_acc_x'], imu['linear_acc_y'], imu['linear_acc_z'],
                imu['angular_vel_x'], imu['angular_vel_y'], imu['angular_vel_z'],
                act_values['rudder'], act_values['propeller'],
                odom['position_x'], odom['position_y']
            ])
    
    combined_file.close()
    
    # Create smoothed combined dataset
    smoothed_combined_file = open(csv_files['smoothed_combined'], 'a', newline='')
    smoothed_combined_writer = csv.writer(smoothed_combined_file)
    
    # For each actuator timestamp, find the closest IMU and smoothed odometry data
    for act_timestamp, act_values in actuator_data.items():
        # Find closest IMU timestamp
        closest_imu_timestamp = min(imu_data.keys(), key=lambda x: abs(x - act_timestamp), default=None)
        
        # Find closest smoothed odometry timestamp
        closest_smooth_odom_timestamp = min(smoothed_odom_data.keys(), key=lambda x: abs(x - act_timestamp), default=None) if smoothed_odom_data else None
        
        if closest_imu_timestamp and closest_smooth_odom_timestamp:
            # Get the corresponding data
            imu = imu_data[closest_imu_timestamp]
            smooth_odom = smoothed_odom_data[closest_smooth_odom_timestamp]
            
            # Write to smoothed combined CSV file
            smoothed_combined_writer.writerow([
                act_timestamp,
                imu['linear_acc_x'], imu['linear_acc_y'], imu['linear_acc_z'],
                imu['angular_vel_x'], imu['angular_vel_y'], imu['angular_vel_z'],
                act_values['rudder'], act_values['propeller'],
                smooth_odom['position_x'], smooth_odom['position_y']
            ])
    
    smoothed_combined_file.close()
    
    print(f"Processed bag: {bag_path}")
    print(f"Data saved to {output_dir}/{bag_name}_*.csv")
    
    # Return statistics about the data
    return {
        'bag': bag_name,
        'imu_samples': len(imu_data),
        'odom_samples': len(odom_data),
        'smoothed_odom_samples': len(smoothed_odom_data),
        'actuator_samples': len(actuator_data),
        'combined_samples': len(actuator_data),  # Same as actuator since we sync on actuator timestamps
        'smoothed_combined_samples': sum(1 for _ in open(csv_files['smoothed_combined'])) - 1  # Subtract 1 for header
    }

def process_all_bags(base_dir, output_dir, min_distance=0.3):
    """Process all bag files in subdirectories of the given directory"""
    # Initialize ROS context
    rclpy.init()
    
    # Find all bag files
    bag_files = []
    for root, dirs, files in os.walk(base_dir):
        if any(f.endswith('.db3') for f in files):
            db3_file = next(f for f in files if f.endswith('.db3'))
            bag_files.append(os.path.join(root, db3_file))
    
    stats = []
    for bag_file in bag_files:
        try:
            stat = process_bag(bag_file, output_dir, min_distance=min_distance)
            stats.append(stat)
        except Exception as e:
            print(f"Error processing {bag_file}: {str(e)}")
    
    # Print statistics
    print("\nData Extraction Statistics:")
    print("=" * 100)
    print(f"{'Bag Name':<20} | {'IMU':<8} | {'Odom':<8} | {'Smooth Odom':<12} | {'Actuator':<10} | {'Combined':<10} | {'Smooth Comb.':<12}")
    print("-" * 100)
    
    for stat in stats:
        print(f"{stat['bag']:<20} | {stat['imu_samples']:<8} | {stat['odom_samples']:<8} | "
              f"{stat['smoothed_odom_samples']:<12} | {stat['actuator_samples']:<10} | "
              f"{stat['combined_samples']:<10} | {stat['smoothed_combined_samples']:<12}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Extract data from ROS2 bags')
    parser.add_argument('--base_dir', type=str, default='../data_from_test', 
                        help='Base directory containing ROS2 bags')
    parser.add_argument('--output_dir', type=str, default=None, 
                        help='Output directory for extracted data')
    parser.add_argument('--min_distance', type=float, default=0.3, 
                        help='Minimum distance between positions in meters')
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
    process_all_bags(base_dir, output_dir, min_distance=args.min_distance) 