import os

from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction

from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_dir = get_package_share_directory('robot2023')
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='robot2023',
            executable='generate_map',
            name='generate_map',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"rayMarchResolution": 0.05},
            ]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename' : os.path.join(my_dir, "maps", "Mapirlab", "occupancy.yaml")},
                {'frame_id' : 'map'}
                ],
            ),
        # LIFECYCLE MANAGER
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': [
                            'map_server',
                            ]
                        }
            ]
        ),
    ])