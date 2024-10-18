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
import signal
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))
import module_kinematics as kin

app = Flask(__name__, static_folder='static')
trajectory = []
encoders = []

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Dictionary to keep track of active ROS 2 launch processes
ros2_processes = {}
process_lock = threading.Lock()  # Lock for thread-safe access to ros2_processes

def launch_ros2_node(package, launch_file):
    command = ['ros2', 'launch', package, launch_file]
    try:
        # Start the ROS 2 launch process
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        logging.info(f'Launched {launch_file} from package {package} with PID {process.pid}')
        return process
    except Exception as e:
        logging.error(f'Failed to launch {launch_file} from package {package}: {str(e)}')
        return str(e)

def stop_process_with_failsafe(process, timeout=5):
    try:
        # Attempt graceful termination
        # process.terminate()
        # os.kill(process.pid, signal.SIGTERM)
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        
        start_time = time.time()
        while process.poll() is None:
            if time.time() - start_time > timeout:
                process.kill()
                break
            time.sleep(0.1)
        process.wait(timeout=1)
        logging.info(f'Process with PID {process.pid} terminated successfully')
    except Exception as e:
        logging.error(f'Failed to terminate process with PID {process.pid}: {str(e)}')
        return f'Failed to terminate process: {str(e)}'
    return None

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
    package_launch_list = data.get('package_launch')
    # package_launch_list = request.form.getlist('package_launch')
    
    if not package_launch_list:
        return jsonify({'error': 'No package-launch pairs selected'}), 400

    responses = []
    with process_lock:
        for package_launch in package_launch_list:
            package, launch_file = package_launch.split(':')

            if (package, launch_file) in ros2_processes:
                responses.append({'error': f'{package} - {launch_file} is already running'})
                continue

            process = launch_ros2_node(package, launch_file)
            if isinstance(process, str):
                responses.append({'error': f'Failed to launch {package} - {launch_file}: {process}'})
            else:
                ros2_processes[(package, launch_file)] = process
                responses.append({'message': f'Launched {package} - {launch_file}', 'pid': process.pid})

    return jsonify(responses), 200

@app.route('/stop_ros2', methods=['POST'])
def stop_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    with process_lock:
        process_key = (package, launch_file)
        if process_key not in ros2_processes:
            return jsonify({'error': f'{package} - {launch_file} is not running'}), 400

        process = ros2_processes[process_key]
        error = stop_process_with_failsafe(process)
        if error:
            return jsonify({'error': error}), 500

        del ros2_processes[process_key]
    return jsonify({'message': f'Stopped {package} - {launch_file}'}), 200

@app.route('/active_launches', methods=['GET'])
def active_launches():
    with process_lock:
        active_list = [
            {'package': pkg, 'launch_file': lf, 'pid': proc.pid}
            for (pkg, lf), proc in ros2_processes.items()
        ]
    return jsonify(active_list), 200

@app.route('/reset_ros2', methods=['POST'])
def reset_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    process_key = (package, launch_file)
    with process_lock:
        if process_key not in ros2_processes:
            return jsonify({'error': f'{package} - {launch_file} is not running'}), 400

        # Stop the existing process
        process = ros2_processes[process_key]
        error = stop_process_with_failsafe(process)
        if error:
            return jsonify({'error': error}), 500

        # Restart the process
        new_process = launch_ros2_node(package, launch_file)
        if isinstance(new_process, str):
            return jsonify({'error': f'Failed to restart {package} - {launch_file}: {new_process}'}), 500

        # Update the process dictionary
        ros2_processes[process_key] = new_process

    return jsonify({'message': f'Reset {package} - {launch_file} successfully', 'pid': new_process.pid}), 200

def main():
    global node, t0, odom_topic, encoders_topic

    odom_topic = '/makara_00/odometry'  # Default selected topic
    encoders_topic = '/makara_00/encoders'  # Default selected topic

    rclpy.init()
    node = OdometrySubscriber()
    now = node.get_clock().now().to_msg()
    t0 = now.sec + now.nanosec * 1.0e-9
    
    start_flask()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()