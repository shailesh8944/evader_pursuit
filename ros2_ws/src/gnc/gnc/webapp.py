import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator
from geometry_msgs.msg import Point
from flask import Flask, render_template, jsonify, request
import threading
import logging
from logging.handlers import RotatingFileHandler
import subprocess
import time

import sys
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))
import module_kinematics as kin


# Dictionary to keep track of active ROS 2 launch processes
ros2_processes = {}

# Function to launch a ROS 2 node
def launch_ros2_node(package, launch_file):
    command = ['ros2', 'launch', package, launch_file]
    try:
        # Start the ROS 2 launch process
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return process
    except Exception as e:
        return str(e)

# Function to safely stop a process with failsafes
def stop_ros2_with_failsafe(process, timeout=5):
    try:
        # Attempt graceful termination
        process.terminate()
        start_time = time.time()
        while process.poll() is None:  # Process is still running
            if time.time() - start_time > timeout:
                # Timeout exceeded, escalate to forceful termination
                process.kill()
                break
            time.sleep(0.1)  # Wait a bit before checking again
        # Ensure process is actually terminated
        process.wait(timeout=1)
    except Exception as e:
        return f'Failed to terminate process: {str(e)}'
    return None


app = Flask(__name__, static_folder='static')
trajectory = []
encoders = []

def odometry_callback(msg):
    global trajectory, t0

    t = np.float64(msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9) - t0

    position = msg.pose.pose.position
    
    orientation = msg.pose.pose.orientation
    quat = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
    eul = kin.quat_to_eul(quat, deg=True)
    
    vel = msg.twist.twist.linear
    
    ang_vel = msg.twist.twist.angular
    
    trajectory.append([
            t,
            vel.x, vel.y, vel.z,
            ang_vel.x, ang_vel.y, ang_vel.z,
            position.x, position.y, position.z,
            eul[0], eul[1], eul[2]
        ])

def encoders_callback(msg):
    global encoders, t0

    t = np.float64(msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9) - t0

    rudder = msg.rudder
    propeller = msg.propeller
    
    encoders.append([t, rudder, propeller])

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            odometry_callback,
            10
        )
        self.encoders_subscription = self.create_subscription(
            Actuator,
            encoders_topic,
            encoders_callback,
            10
        )

def flask_thread():
    app.run(host='0.0.0.0', port=8500)

def start_flask():
    app_thread = threading.Thread(target=flask_thread)
    app_thread.start()

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/odometry_plot')
def odometry_plot():
    return render_template('odometry_plot.html')

@app.route('/mission')
def mission():
    return render_template('mission.html')

@app.route('/odometry')
def odometry():
    return render_template('odometry.html')

@app.route('/get_trajectory')
def get_trajectory():
    global trajectory
    return jsonify(trajectory)

@app.route('/get_encoders')
def get_encoders():
    global encoders
    return jsonify(encoders)

@app.route('/get_state')
def get_state():
    global trajectory
    global encoders
    dict = {}
    dict['odometry'] = trajectory[-1]
    dict['encoders'] = encoders[-1]
    return jsonify(dict)

@app.route('/reset', methods=['POST'])
def reset():
    global trajectory, encoders
    trajectory = []  # Reset the trajectory variable
    encoders = []  # Reset the encoders variable
    return 'Trajectory and encoders variables reset'

@app.route('/start_ros2', methods=['POST'])
def start_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    # Check if a launch is already running for this package
    if (package, launch_file) in ros2_processes:
        return jsonify({'error': 'This ROS 2 launch is already running'}), 400

    # Start the ROS 2 launch
    process = launch_ros2_node(package, launch_file)
    if isinstance(process, str):  # If the process is an error message
        return jsonify({'error': process}), 500

    # Store the process
    ros2_processes[(package, launch_file)] = process
    return jsonify({'message': f'Launched {launch_file} from package {package}', 'pid': process.pid}), 200

@app.route('/stop_ros2', methods=['POST'])
def stop_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    # Check if the process is running
    process_key = (package, launch_file)
    if process_key not in ros2_processes:
        return jsonify({'error': 'This ROS 2 launch is not running'}), 400

    # Terminate the process with failsafes
    process = ros2_processes[process_key]
    error = stop_ros2_with_failsafe(process)
    if error:
        return jsonify({'error': error}), 500

    # Remove from dictionary after successful termination
    del ros2_processes[process_key]
    return jsonify({'message': f'Stopped {launch_file} from package {package}'}), 200

@app.route('/reset_ros2', methods=['POST'])
def reset_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    # Stop the existing process if it is running
    process_key = (package, launch_file)
    if process_key in ros2_processes:
        process = ros2_processes[process_key]
        error = stop_ros2_with_failsafe(process)
        if error:
            return jsonify({'error': error}), 500
        # Remove from dictionary after successful termination
        del ros2_processes[process_key]

    # Start a new process
    process = launch_ros2_node(package, launch_file)
    if isinstance(process, str):  # If the process is an error message
        return jsonify({'error': process}), 500

    # Store the new process
    ros2_processes[process_key] = process
    return jsonify({'message': f'Restarted {launch_file} from package {package}', 'pid': process.pid}), 200

def main():
    global node, t0, odom_topic, encoders_topic

    odom_topic = '/makara_00/odometry'  # Default selected topic
    encoders_topic = '/makara_00/actuator_cmd'  # Default selected topic

    rclpy.init()
    node = OdometrySubscriber()
    now = node.get_clock().now().to_msg()
    t0 = now.sec + now.nanosec * 1.0e-9
    
    start_flask()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()