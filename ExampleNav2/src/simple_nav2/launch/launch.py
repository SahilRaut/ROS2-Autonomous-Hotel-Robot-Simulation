import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete

def generate_launch_description():

    # Incoming parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    use_composition = LaunchConfiguration('use_composition')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Path variables
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    simple_nav2_dir = os.path.join(get_package_share_directory('simple_nav2'), 'launch')
    
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('simple_nav2'),
            'params',
            'simple_nav2_params.yaml'))

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('simple_nav2'),
            'map',
            'map.yaml'))

    rviz_config_file = os.path.join(
        get_package_share_directory('simple_nav2'),
        'config',
        'simple_nav2.rviz')
    
    xacro_path = os.path.join(
        get_package_share_directory('simple_robot_description'),
        'urdf',
        'simple_robot_gazebo.urdf.xacro')
    
    lifecycle_nodes = ['map_server', 'amcl']    
    
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[{'yaml_filename': map_dir,
                            'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn_delay=2.0,
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Boolean to indicate use of sim time'),

        # Static transform publisher - for base_footprint
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = 'base_footprint_to_base_link',
            arguments = ['0.', '0.', '0.', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        # Robot state publisher
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
        
        # Group nodes
        load_nodes,

       # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'params': param_dir,
                }.items(),
        ),

       # Control node
        Node(
            package='simple_nav2',
            executable='simple_nav2',
            name='simple_nav2',
            output='screen'
        ),
    ])
