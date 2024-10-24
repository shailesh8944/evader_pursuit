import numpy as np
from scipy.io import loadmat

def import_ros2_mat_file(mat_file_path):
    # Load the .mat file
    mat_data = loadmat(mat_file_path)

    # Extract the Imu data
    imu_data = {
        'time': mat_data['Imu']['time'][0][0].flatten(),
        'orientation': mat_data['Imu']['orientation'][0][0],
        'angular_velocity': mat_data['Imu']['angular_velocity'][0][0],
        'linear_acceleration': mat_data['Imu']['linear_acceleration'][0][0]
    }

    # Extract the NavSatFix data
    navsat_data = {
        'time': mat_data['NavSatFix']['time'][0][0].flatten(),
        'latitude': mat_data['NavSatFix']['latitude'][0][0].flatten(),
        'longitude': mat_data['NavSatFix']['longitude'][0][0].flatten(),
        'altitude': mat_data['NavSatFix']['altitude'][0][0].flatten()
    }

    # Print some information to confirm the import
    print("Imu Data:")
    print(f"Time: {imu_data['time'][:5]}...")  # Display the first 5 time values
    print(f"Orientation: {imu_data['orientation'][:5]}...")  # First 5 orientation entries
    print(f"Angular Velocity: {imu_data['angular_velocity'][:5]}...")
    print(f"Linear Acceleration: {imu_data['linear_acceleration'][:5]}...")

    print("\nNavSatFix Data:")
    print(f"Time: {navsat_data['time'][:5]}...")  # Display the first 5 time values
    print(f"Latitude: {navsat_data['latitude'][:5]}...")
    print(f"Longitude: {navsat_data['longitude'][:5]}...")
    print(f"Altitude: {navsat_data['altitude'][:5]}...")

    return imu_data, navsat_data

if __name__ == '__main__':
    mat_file_path = 'test_01/ros2_data.mat'  # Replace with your .mat file path
    imu_data, navsat_data = import_ros2_mat_file(mat_file_path)
