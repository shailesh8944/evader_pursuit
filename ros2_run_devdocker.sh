if [ $# -eq 0 ]
  then
    echo "No arguments supplied! bash assumed"
    cmd=bash
  else
    cmd=$1
fi

docker run -it --rm --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="ROS_DOMAIN_ID=42" \
    --workdir="/workspaces/makara" \
    --volume="$(pwd)":"/workspaces/makara" \
    --volume="/dev/shm":"/dev/shm" \
    --volume="/dev/ttyACM0":"/dev/ttyACM0" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/mav_simulator_ros2:1.0 $cmd
