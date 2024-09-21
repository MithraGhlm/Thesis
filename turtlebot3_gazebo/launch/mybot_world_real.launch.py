#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    package_name = 'turtlebot3_gazebo' 
    launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    my_package_dir = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-8.5')
    y_pose = LaunchConfiguration('y_pose', default='2.0')

   ## replace Ur world here to make it appear
    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'room_with_objects_2.world'
    )


    ## robot state publisher launch file
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'mybot_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control':use_ros2_control}.items()
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



    # ask the robot_state_publisher node to return robot_description parameter as a string
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    # launch ros2_controll_node, set its robot_description parameter, and take other parameters specified in the yaml file.
    controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


    diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rviz_laucher)
    ld.add_action(twist_mux)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_diff_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    #ld.add_action(diff_drive_spawner)
    #ld.add_action(joint_broad_spawner)

    return ld
