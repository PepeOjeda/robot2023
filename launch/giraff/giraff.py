import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction

from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        Node(
            package='robot2023',
            executable='reactive_follower',
            name='reactive_follower',
            namespace="giraff",
            output='screen',
            parameters=[
                {"/follower/linearSpeed" : 0.3},
                {"/follower/directionTolerance" : 0.1},
                {"/follower/local_frame_id" : "giraff_base_link"},
                {"/follower/master_loc_topic" : "/rhodon/status"},
            ]  
        ),

        Node(
            package='nav2_over_mqtt',
            executable='nav2MqttSender',
            name='nav2MqttSender',
            output='screen',
            namespace="giraff",
            parameters=[
                {"goalTopic":"/giraff/NavToPose"},
                {"resultTopic":"/giraff/NavigationResult"},
            ]  
        ),
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[
                {"host":"150.214.109.137"},
                {"port":8002},
                {"MQTT_namespace":"pc"},
                {"MQTT_topics_subscribe":"/giraff/NavigationResult"},
            ]            
        ),
    ])