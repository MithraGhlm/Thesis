# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try running the script as root.
## then run this script.
xhost local:root

XAUTH=/tmp/.docker.xauth

sudo docker run -it\
    --name=humble_gzclassic_3_rviz \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/mithra/BFH/Thesis/docker_thesis/ros2_thesis_ws:/ros2_ws" \
    --volume="/dev:/dev" \
    --volume="/var/run/dbus:/var/run/dbus" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    --privileged \
    ros2_humble_gazebo11_thesis_3 \
    bash

echo "Done."
