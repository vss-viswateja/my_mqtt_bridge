#!/usr/bin/env python3
"""
MQTT to ROS 2 Nav2 Bridge Node

This node bridges MQTT commands from a web UI to ROS 2 Nav2 action goals
for a multi-robot fleet simulation.

Author: Senior ROS 2 Developer
Date: November 2025
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import paho.mqtt.client as mqtt


class MQTTBridgeNode(Node):
    """
    Bridge node that converts MQTT navigation commands to Nav2 action goals.
    
    This node subscribes to an MQTT topic and publishes Nav2 NavigateToPose
    action goals to individual robot namespaces based on the robot_id in
    the received command.
    """

    def __init__(self):
        """Initialize the MQTT Bridge Node."""
        super().__init__('mqtt_bridge_node')
        
        # Declare and get parameters
        self.declare_parameter('mqtt_broker', 'test.mosquitto.org')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'fleet/commands/navigate')
        
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        
        # Define the robot fleet
        self.robot_names = [
            'SmallDeliveryRobot_0',
            'SmallDeliveryRobot_1',
            'SmallDeliveryRobot_2',
            'SmallDeliveryRobot_3'
        ]
        
        # Create a dictionary of Action Clients, one per robot
        self.action_clients = {}
        for robot_name in self.robot_names:
            action_name = f'/{robot_name}/navigate_to_pose'
            client = ActionClient(self, NavigateToPose, action_name)
            self.action_clients[robot_name] = client
            self.get_logger().info(f'Created action client for {action_name}')
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(
            client_id=f'ros2_bridge_{self.get_name()}',
            clean_session=True
        )
        
        # Set MQTT callbacks
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        self.get_logger().info(
            f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}...'
        )
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info('MQTT client loop started')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """
        Callback for when the MQTT client connects to the broker.
        
        Args:
            client: The MQTT client instance
            userdata: User-defined data
            flags: Response flags from the broker
            rc: Connection result code
        """
        if rc == 0:
            self.get_logger().info('Successfully connected to MQTT broker')
            # Subscribe to the navigation command topic
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'Subscribed to MQTT topic: {self.mqtt_topic}')
        else:
            self.get_logger().error(f'MQTT connection failed with code: {rc}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        """
        Callback for when the MQTT client disconnects from the broker.
        
        Args:
            client: The MQTT client instance
            userdata: User-defined data
            rc: Disconnection result code
        """
        if rc != 0:
            self.get_logger().warn(f'Unexpected MQTT disconnection (code: {rc})')
        else:
            self.get_logger().info('Disconnected from MQTT broker')

    def on_mqtt_message(self, client, userdata, msg):
        """
        Callback for when an MQTT message is received.
        
        This is the core logic that:
        1. Decodes and parses the JSON command
        2. Extracts robot_id and goal coordinates
        3. Builds a NavigateToPose goal message
        4. Sends the goal to the appropriate robot's action server
        
        Args:
            client: The MQTT client instance
            userdata: User-defined data
            msg: The received MQTT message
        """
        try:
            # Decode the message payload from bytes to string
            payload_str = msg.payload.decode('utf-8')
            self.get_logger().info(f'Received MQTT message: {payload_str}')
            
            # Parse the JSON command
            command = json.loads(payload_str)
            
            # Extract robot_id and goal
            robot_id = command.get('robot_id')
            goal_data = command.get('goal')
            
            if not robot_id or not goal_data:
                self.get_logger().error(
                    'Invalid command format: missing robot_id or goal'
                )
                return
            
            # Validate robot_id
            if robot_id not in self.action_clients:
                self.get_logger().error(
                    f'Unknown robot_id: {robot_id}. '
                    f'Valid robots are: {self.robot_names}'
                )
                return
            
            # Extract coordinates
            x = float(goal_data.get('x', 0.0))
            y = float(goal_data.get('y', 0.0))
            
            self.get_logger().info(
                f'Parsed command: Robot={robot_id}, Goal=({x}, {y})'
            )
            
            # Build the NavigateToPose goal message
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose.header.frame_id = 'map'
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            nav_goal.pose.pose.position.x = x
            nav_goal.pose.pose.position.y = y
            nav_goal.pose.pose.position.z = 0.0
            nav_goal.pose.pose.orientation.x = 0.0
            nav_goal.pose.pose.orientation.y = 0.0
            nav_goal.pose.pose.orientation.z = 0.0
            nav_goal.pose.pose.orientation.w = 1.0  # No rotation (facing forward)
            
            # Get the appropriate action client
            action_client = self.action_clients[robot_id]
            
            # Wait for the action server to be available (with timeout)
            if not action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    f'Action server for {robot_id} not available yet'
                )
            
            # Send the goal asynchronously
            self.get_logger().info(
                f'Sending navigation goal to {robot_id}: ({x}, {y})'
            )
            
            send_goal_future = action_client.send_goal_async(
                nav_goal,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(
                lambda future: self.goal_response_callback(future, robot_id)
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except ValueError as e:
            self.get_logger().error(f'Invalid coordinate values: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')

    def goal_response_callback(self, future, robot_id):
        """
        Callback for when the action server responds to a goal request.
        
        Args:
            future: The future object containing the goal handle
            robot_id: The ID of the robot that received the goal
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(
                    f'Goal rejected by {robot_id}'
                )
                return
            
            self.get_logger().info(
                f'Goal accepted by {robot_id}, navigating...'
            )
            
            # Optionally, wait for the result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self.result_callback(future, robot_id)
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in goal response: {e}')

    def feedback_callback(self, feedback_msg):
        """
        Callback for action feedback (optional).
        
        Args:
            feedback_msg: The feedback message from the action server
        """
        # You can log feedback here if needed
        pass

    def result_callback(self, future, robot_id):
        """
        Callback for when the navigation action completes.
        
        Args:
            future: The future object containing the result
            robot_id: The ID of the robot that completed the action
        """
        try:
            result = future.result().result
            self.get_logger().info(
                f'{robot_id} completed navigation action'
            )
        except Exception as e:
            self.get_logger().error(
                f'{robot_id} navigation failed: {e}'
            )

    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.get_logger().info('Shutting down MQTT bridge node...')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point for the MQTT bridge node."""
    rclpy.init(args=args)
    
    node = MQTTBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
