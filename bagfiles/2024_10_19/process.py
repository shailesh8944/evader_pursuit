import numpy as np
import matplotlib.pyplot as plt
from show_extraction import import_ros2_mat_file

mat_file_path = 'test_01/ros2_data.mat'
imu_data, navsat_data = import_ros2_mat_file(mat_file_path)

t_imu = imu_data['time']
q_ni = imu_data['orientation']
w_ni_i = imu_data['angular_velocity']
a_ni_i = imu_data['linear_acceleration']

t_gps = navsat_data['time']
lat = navsat_data['latitude']
lon = navsat_data['longitude']
alt = navsat_data['altitude']

