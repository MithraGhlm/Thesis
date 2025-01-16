#!/bin/bash

CONTAINER_ID=$(docker ps -a | grep humble_gzclassic_3_rviz | cut -f 1 -d " ")
docker start $CONTAINER_ID
gnome-terminal --tab -- bash -c "docker attach $CONTAINER_ID bash -c 'cd ros2_ws && source install/setup.bash"

for i in {1..6}; do
	gnome-terminal --tab -- bash -c "docker exec -it $CONTAINER_ID bash -c 'cd ros2_ws && source install/setup.bash && exec bash'"
done

