"""
Complete MQTT-Controlled Multi-Robot Fleet Launch File

This launch file:
1. Launches the multi-robot Nav2 simulation
2. Starts the MQTT bridge node for web-based control

Usage:
    ros2 launch my_mqtt_bridge fleet_with_mqtt_bridge.launch.py

Then open index.html in your browser to control the robots via MQTT.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for complete MQTT-controlled fleet.
    """
    
    # Path to the multi-robot navigation launch file
    multi_robot_nav_launch = PathJoinSubstitution([
        FindPackageShare('my_mqtt_bridge'),
        'launch',
        'multi_robot_nav.launch.py'
    ])
    
    # Include the multi-robot navigation launch file
    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([multi_robot_nav_launch]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': 'simple',
            'map': 'simple',
            'number_of_humans': '0',
            'sample': '0',
        }.items()
    )
    
    # Start MQTT bridge node after a delay to ensure Nav2 is ready
    mqtt_bridge_node = TimerAction(
        period=10.0,  # Wait 10 seconds for Nav2 to initialize
        actions=[
            Node(
                package='my_mqtt_bridge',
                executable='bridge_node',
                name='mqtt_bridge',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'mqtt_broker': 'test.mosquitto.org'},
                    {'mqtt_port': 1883},
                    {'mqtt_topic': 'fleet/commands/navigate'}
                ]
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add all actions
    ld.add_action(start_simulation)
    ld.add_action(mqtt_bridge_node)
    
    return ld
