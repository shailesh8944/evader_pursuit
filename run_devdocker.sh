docker run -it --rm --privileged --name mav_simulator \
    --net=host \
    --env="DISPLAY" \
    --workdir="/home/mavlab/colcon_ws/src/mav_simulator/mav_simulator" \
    --volume="$(pwd)/ros2_ws":"/home/mavlab/colcon_ws" \
    --volume="/dev/ttyACM0":"/dev/ttyACM0" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/mav_simulator:1.0 bash
