cd ros2_ws
colcon build
source /opt/ros/humble/setup.bash && source /workspaces/makara/ros2_ws/install/setup.bash
ros2 run mav_simulator simulate