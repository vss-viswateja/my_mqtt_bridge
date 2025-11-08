"""
MQTT Bridge Only Launch File

Launches only the MQTT bridge node for connecting web UI to existing Nav2 robots.
Use this when your simulation is already running.

Usage:
    ros2 launch my_mqtt_bridge mqtt_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for MQTT bridge node only."""
    
    # Declare launch arguments
    declare_mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='test.mosquitto.org',
        description='MQTT broker hostname'
    )
    
    declare_mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    declare_mqtt_topic_arg = DeclareLaunchArgument(
        'mqtt_topic',
        default_value='fleet/commands/navigate',
        description='MQTT topic to subscribe to for navigation commands'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time'
    )
    
    # MQTT Bridge Node
    mqtt_bridge_node = Node(
        package='my_mqtt_bridge',
        executable='bridge_node',
        name='mqtt_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'mqtt_broker': LaunchConfiguration('mqtt_broker')},
            {'mqtt_port': LaunchConfiguration('mqtt_port')},
            {'mqtt_topic': LaunchConfiguration('mqtt_topic')}
        ]
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_mqtt_broker_arg)
    ld.add_action(declare_mqtt_port_arg)
    ld.add_action(declare_mqtt_topic_arg)
    ld.add_action(declare_use_sim_time_arg)
    
    # Add node
    ld.add_action(mqtt_bridge_node)
    
    return ld
