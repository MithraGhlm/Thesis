# Autonomous Mobile Robot (Caspi)
## Demo
You can find some videos of tests in the Demo folder.

https://github.com/user-attachments/assets/b185de21-92a6-49ba-8590-1895bb96ea0f

## Directory structure

`Utils` : Scripts, data and tools to build and run the project <br>
`Report` : Final thesis report pdf<br>
`Report-updated` : An updated version of the final thesis report pdf<br>

The rest of the directories are ROS2 packages.

# Building

Install ROS2 humble, Gazebo 11, and other dependencies by looking at `Dockerfile` included in `Utils` directory.

1. Make a ROS2 workspace
2. Create a `src` directory.
3. Clone the content of this repository into `src`
4. Go back to ROS2 workspace and run `colcon build`
5. Refer to report appendix to run the project.


# Running the project

A detailed guide to running the project is in the project Report, part Apendix.<br>
As a review, the following nodes should be running for the whole system to work.<br>

The main node:<br>
```$ ros2 launch caspi_bot caspibot_world_real.launch.py```<br>

The LiDAR node:<br>
```$ ros2 launch urg_node urg_node_launch.py```

The joystick node:<br>
```$ ros2 launch caspi_bot joystick_caspibot.launch.py```

To capture a map from the environment:<br>
```$ ros2 launch caspi_bot online_async_launch.py params_file:=./src/caspi_bot/config mapper_params_online_async.yaml use_sim_time:=false```

To save the map:<br>
```$ ros2 run nav2_map_server map_saver_cli -f path/to/save/<maps_name>```

Run localization in a pre-saved map:<br>
```$ ros2 launch caspi_bot localization_launch.py map:=path/to/saved/map/<maps_name>.yaml use_sim_time:=false```

To navigate in the captured map:<br>
```$ ros2 launch caspi_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true```

Run collision monitor:<br>
```$ ros2 launch caspi_bot collision_monitor_node.launch.py```

Waypoint following node:<br>
```$ ros2 run waypoint_navigation follow_waypoints```

Detecting the Station and Docking to it:<br>
```$ ros2 run station_docking detect_docking``` <br>
```$ ros2 run station_docking attach_docking```
