docker run -it --rm --privileged --name raspi_docker\
    --net=host \
    --env="DISPLAY" \
    --workdir="/workspaces/mavlab" \
    --volume="$(pwd)":"/workspaces/mavlab" \
    --volume="/dev/sbg":"/dev/sbg" \
    --volume="/dev/ardusimple":"/dev/ardusimple" \
    --volume="/dev/bldc":"/dev/bldc" \
    --volume="/dev/stepper":"/dev/stepper" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/mav_simulator_ros1:1.0 ./raspi_entry.sh