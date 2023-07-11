import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    my_dir = get_package_share_directory('robot2023')
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='nav2_over_mqtt',
            executable='nav2MqttSender',
            name='nav2MqttSender',
            output='screen',
            namespace="rhodon",
            parameters=[
                {"goalTopic":"/rhodon/NavToPose"},
                {"resultTopic":"/rhodon/NavigationResult"},
            ]  
        ),

        Node(
            package="robot2023",
            executable="test_node",
            name="test_node",
            output="screen",
            parameters=[
                {"followerDistance":1.5},
            ]  
        ),

        # MAP
        ##############
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', os.path.join(my_dir, 'launch', 'simulation', 'robot2023.rviz')],
            output="log",
            #prefix='xterm -hold -e',
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