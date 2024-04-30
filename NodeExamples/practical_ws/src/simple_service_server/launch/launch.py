import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='simple_service_server',
            executable='server',
            name='server',
            output='screen',
            parameters=[{
            }]
        ),
        Node(
            package='simple_service_client',
            executable='client',
            name='client',
            output='screen',
            parameters=[{
            }]
        ),
    ])
