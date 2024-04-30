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
    # Incoming parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    rviz_config_file = os.path.join(
        get_package_share_directory('simple_nav2'),
        'rviz',
        'simple_nav2.rviz')
    
    xacro_path = os.path.join(
        get_package_share_directory('simple_robot_description'),
        'urdf',
        'simple_robot_gazebo.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Boolean to indicate use of sim time'
        ),

        # Static transform publisher - for base_footprint
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = 'base_footprint_to_base_link',
            arguments = ['0.', '0.', '0.', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description':Command(['xacro',' ', xacro_path])
            }]),

        # RVIZ
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
