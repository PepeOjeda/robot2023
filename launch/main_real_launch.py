import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    my_dir = get_package_share_directory('robot2023')
    
    rviz_giraff =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_giraff',
        arguments=['-d', os.path.join(my_dir, 'launch', 'giraff', 'giraff.rviz')],
        output="log",
        prefix='xterm -hold -e',
        remappings=[
            ("/initialpose", "/giraff/initialpose"),
            ("/goal_pose", "/giraff/goal_pose")
        ]
    )
    rviz_rhodon =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_rhodon',
        arguments=['-d', os.path.join(my_dir, 'launch', 'rhodon', 'rhodon.rviz')],
        output="log",
        prefix='xterm -hold -e',
        remappings=[
            ("/initialpose", "/rhodon/initialpose"),
            ("/goal_pose", "/rhodon/goal_pose")
        ]
    )

    giraff = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot2023'),'launch', 'giraff', 'nav2_launch.py')
        ),
        launch_arguments={
            'namespace': 'giraff',
            'scenario': LaunchConfiguration('scenario'),
            'nav_params_yaml': LaunchConfiguration('nav_params_yaml'),
        }.items()
    )
    rhodon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot2023'),'launch', 'rhodon', 'nav2_launch.py')
        ),
        launch_arguments={
            'namespace': 'rhodon',
            'scenario': LaunchConfiguration('scenario'),
            'nav_params_yaml': LaunchConfiguration('nav_params_yaml'),
        }.items()
    )

    return[
        rviz_giraff,
        rviz_rhodon,
        giraff,
        rhodon,
    ]

def generate_launch_description():

    my_dir = get_package_share_directory('robot2023')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument('scenario', default_value="Mapirlab"), #required
        DeclareLaunchArgument('nav_params_yaml', default_value=os.path.join(my_dir, 'launch', 'nav2_params.yaml') ),
        OpaqueFunction(function = launch_setup)
    ])
