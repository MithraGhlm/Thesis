#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'turtlebot3_gazebo' 
    launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    my_package_dir = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-8.5')
    y_pose = LaunchConfiguration('y_pose', default='2.0')

   ## replace Ur world here to make it appear
    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'room_with_objects_2.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )


    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ) ,
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    ## robot state publisher launch file
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'mybot_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control':use_ros2_control}.items()
    )


    ## launch file to spawn Ur own robot
    spawn_mybot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_mybot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    if use_sim_time == 'true':
        _use_sim_time = f"use_sim_time:={True}"
    else:
        _use_sim_time = f"use_sim_time:={False}"
        
    rviz_laucher= Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', os.path.join(my_package_dir, 'config', 'main.rviz'), '--ros-args', '-p', _use_sim_time])
    


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_mybot_cmd)
    # -------
    ld.add_action(rviz_laucher)
    ld.add_action(twist_mux)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    return ld
