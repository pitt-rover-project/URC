import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_name = 'my_robot.urdf'
    urdf = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf, '-entity', 'my_robot'],
            output='screen'),
    ])
