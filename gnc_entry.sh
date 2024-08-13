cd /workspaces/makara/ros2_ws
colcon build
source /opt/ros/humble/setup.bash && source /workspaces/makara/ros2_ws/install/setup.bash
python3 /workspaces/makara/ros2_ws/src/gnc/gnc/subscribe_odometry.py &
ros2 run gnc gnc