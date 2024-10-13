docker run -it --rm --privileged --name raspi_docker\
    --net=host \
    --env="DISPLAY" \
    --workdir="/workspaces/mavlab" \
    --volume="$(pwd)":"/workspaces/mavlab" \
    --volume="/dev/ttyACM0":"/dev/ttyACM0" \
    --volume="/dev/ttyACM2":"/dev/ttyACM2" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/mav_simulator_ros1:1.0 ./raspi_entry.sh