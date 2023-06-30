# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    my_dir = get_package_share_directory('robot2023')

    # common variables
    use_sim_time = True
    remappings=[]
    params_yaml_file = ParameterFile( os.path.join(my_dir, 'launch', 'rhodon', 'nav2_params.yaml'), allow_substs=True)
    map_file = os.path.join(my_dir, 'maps', 'Mapirlab', 'occupancy.yaml')
    
    logger = LaunchConfiguration("log_level")
    
    navigation_nodes = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),

        # MAP_SERVER
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename' : map_file}],
            remappings=remappings
            ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_yaml_file],
            remappings=remappings,
            #prefix='xterm -hold -e',
            arguments=['--ros-args', '--log-level', logger]
            ),

        # BT NAV
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_yaml_file],
            remappings=remappings),

        # PLANNER (global path planning)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_yaml_file],
            remappings=remappings
            ),

        # CONTROLLER (path following / local planber)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_yaml_file],
            remappings=[]
            ),

        # RECOVERIES (recovery behaviours)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_yaml_file],
            remappings=remappings
            ),


        # WAYPOINT NAV (app that launch the Bt navigator)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_yaml_file],
            remappings=remappings),

        # LIFECYCLE MANAGER
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'bt_navigator',
                                        'behavior_server',
                                        'waypoint_follower']
                        }
                       ]
            )
    ]


    actions=[PushRosNamespace(LaunchConfiguration('namespace'))]
    actions.extend(navigation_nodes)

    return LaunchDescription([
        GroupAction
        (
            actions=actions
        )
    ]
    )
