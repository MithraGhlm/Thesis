 #!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time') #, default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control') #, default='true')

    # Get the xacro file path
    xacro_file_name = 'robot.urdf.xacro'
    pkg_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'))
    xacro_file = os.path.join(pkg_path, 'descriptions', xacro_file_name)

    # # Command to process the xacro file and convert it to urdf
    # robot_description_config = Command(['xacro ', xacro_file])
    # params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}

    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}




    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        #robot_state_publisher_node
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    ])

