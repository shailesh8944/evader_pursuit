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
import module_kinematics as kin

app = Flask(__name__)
trajectory = []
encoders = []
odom_topic = '/makara_00/odometry'  # Default selected topic
encoders_topic = '/makara_00/actuator_cmd'  # Default selected topic

# Configure logging
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

# File logging - uncomment if needed
# file_handler = RotatingFileHandler('flask.log', maxBytes=1024 * 1024 * 10, backupCount=5)
# file_handler.setFormatter(formatter)
# file_handler.setLevel(logging.INFO)

# Screen logging
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
stream_handler.setLevel(logging.INFO)

logger = logging.getLogger(__name__)
# logger.addHandler(file_handler)   -> Uncomment if file logging needed
logger.addHandler(stream_handler)
logger.setLevel(logging.INFO)


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
def index():
    return render_template('index.html')

@app.route('/odometry')
def odometry():
    return render_template('odometry.html')

@app.route('/mission')
def mission():
    return render_template('mission.html')

@app.route('/odometry_plot')
def index():
    return render_template('odomometry_chart.html')

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

def main():
    global node, t0
    rclpy.init()
    node = OdometrySubscriber()
    now = node.get_clock().now().to_msg()
    t0 = now.sec + now.nanosec * 1.0e-9
    
    start_flask()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
