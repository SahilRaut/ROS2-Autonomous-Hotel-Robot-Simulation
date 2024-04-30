import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    sdf_path = os.path.join(
        get_package_share_directory('simple_robot_description'),
        'sdf',
        'simple_robot_gazebo.sdf'
    )

    # Incoming parameters
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    return LaunchDescription([
        # Outgoing parameters
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Boolean to indicate use of sim time'),

        DeclareLaunchArgument(
            'x_pose',
            default_value='-3.0',
            description='Initial robot x position'),

        DeclareLaunchArgument(
            'y_pose',
            default_value='1.0',
            description='Initial robot y position'),

        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Gazebo world to use'),

        # Gazebo Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        # Gazebo Client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        # Spawn robot model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_robot',
                '-file', sdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01'
            ],
            output='screen',
        ),
    ])
