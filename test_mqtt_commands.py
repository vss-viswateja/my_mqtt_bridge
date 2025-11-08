#!/usr/bin/env python3
"""
Test script to send MQTT commands to the bridge node.

This script can be used to test the MQTT bridge without needing the web UI.
It sends sample navigation commands to the fleet/commands/navigate topic.

Usage:
    python3 test_mqtt_commands.py
"""

import json
import time
import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, rc):
    """Callback when connected to MQTT broker."""
    if rc == 0:
        print("✓ Connected to MQTT broker")
    else:
        print(f"✗ Connection failed with code: {rc}")


def on_publish(client, userdata, mid):
    """Callback when message is published."""
    print(f"✓ Message published (mid: {mid})")


def send_navigation_command(client, robot_id, x, y):
    """
    Send a navigation command via MQTT.
    
    Args:
        client: MQTT client instance
        robot_id: Name of the robot (e.g., "SmallDeliveryRobot_0")
        x: X coordinate
        y: Y coordinate
    """
    command = {
        "robot_id": robot_id,
        "goal": {
            "x": x,
            "y": y
        }
    }
    
    payload = json.dumps(command)
    topic = "fleet/commands/navigate"
    
    print(f"\nSending command to {robot_id}:")
    print(f"  Topic: {topic}")
    print(f"  Payload: {payload}")
    
    result = client.publish(topic, payload)
    result.wait_for_publish()


def main():
    """Main function to test MQTT commands."""
    print("=" * 60)
    print("MQTT Bridge Test Script")
    print("=" * 60)
    
    # Create MQTT client
    client = mqtt.Client(client_id="test_script")
    client.on_connect = on_connect
    client.on_publish = on_publish
    
    # Connect to broker
    broker = "test.mosquitto.org"
    port = 1883
    
    print(f"\nConnecting to {broker}:{port}...")
    client.connect(broker, port, 60)
    client.loop_start()
    
    # Wait for connection
    time.sleep(2)
    
    # Test commands
    test_cases = [
        ("SmallDeliveryRobot_0", 5.0, 3.0),
        ("SmallDeliveryRobot_1", -2.5, 7.8),
        ("SmallDeliveryRobot_2", 0.0, 0.0),
        ("SmallDeliveryRobot_3", 10.5, -4.2),
    ]
    
    print("\n" + "=" * 60)
    print("Sending test commands...")
    print("=" * 60)
    
    for robot_id, x, y in test_cases:
        send_navigation_command(client, robot_id, x, y)
        time.sleep(1)  # Small delay between commands
    
    print("\n" + "=" * 60)
    print("All test commands sent!")
    print("=" * 60)
    print("\nCheck your ROS 2 bridge node logs to see if commands were received.")
    print("Press Ctrl+C to exit...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
