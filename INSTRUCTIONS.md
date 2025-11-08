# MQTT Robot Fleet Control - Instructions

## 🚀 Quick Start

### Step 1: Start the Web Server

Open a terminal and run:

```bash
cd /home/viswa/robot_ws/src/my_mqtt_bridge
python3 -m http.server 8000
```

Or use the quick script:

```bash
cd /home/viswa/robot_ws/src/my_mqtt_bridge
./start_server.sh
```

**Keep this terminal running!** The server needs to stay active.

### Step 2: Open the Web Interface

Open your web browser (Firefox, Chrome, etc.) and go to:

**Main Control Interface:** http://localhost:8000/index.html

**Test Page (optional):** http://localhost:8000/mqtt_test.html

---

## 🤖 How to Send Commands

### Using the Web UI

1. Open http://localhost:8000/index.html
2. Wait for the status to show **"Connected"** (green)
3. Select a robot from the dropdown (SmallDeliveryRobot_0, _1, _2, _3)
4. Enter X and Y coordinates
5. Click **"Send Goal"**

### Using Python Script

From a terminal:

```bash
cd /home/viswa/robot_ws/src/my_mqtt_bridge
python3 test_mqtt_commands.py
```

### Using ROS 2 MQTT Bridge

Make sure your ROS 2 bridge node is running:

```bash
ros2 launch my_mqtt_bridge mqtt_bridge.launch.py
```

Or with the full fleet:

```bash
ros2 launch my_mqtt_bridge fleet_with_mqtt_bridge.launch.py
```

---

## 📡 MQTT Connection Details

- **Broker:** test.mosquitto.org
- **Port:** 8081 (WebSocket Secure)
- **Topic:** `fleet/commands/navigate`
- **Protocol:** WebSocket (wss://)

### Message Format

```json
{
    "robot_id": "SmallDeliveryRobot_0",
    "goal": {
        "x": 5.0,
        "y": 3.0
    }
}
```

---

## 🔧 Troubleshooting

### Web UI shows "Connecting..." forever

**Problem:** MQTT broker not reachable

**Solution:**
- Check internet connection
- Check firewall settings
- Try test page: http://localhost:8000/mqtt_test.html

### "ERROR: MQTT library failed to load"

**Problem:** Opened HTML file directly (double-clicked)

**Solution:** 
- Always use the web server: `python3 -m http.server 8000`
- Access via http://localhost:8000/index.html (not file://)

### Port 8000 already in use

**Problem:** Server already running

**Solution:**
```bash
# Stop existing server
pkill -f "python3 -m http.server 8000"

# Start new server
python3 -m http.server 8000
```

Or use a different port:
```bash
python3 -m http.server 8001
# Then open: http://localhost:8001/index.html
```

### ROS 2 bridge not receiving messages

**Problem:** Bridge not subscribed to MQTT topic

**Solution:**
- Check bridge_node.py is running
- Verify both web UI and bridge use same broker and topic
- Check ROS 2 logs: `ros2 topic echo /robot_commands`

---

## 📁 Files Overview

- **index.html** - Main robot control interface
- **mqtt_test.html** - Connection testing page  
- **test_lib.html** - Library diagnostic page
- **paho-mqtt.min.js** - MQTT client library (local copy)
- **start_server.sh** - Quick server startup script
- **test_mqtt_commands.py** - Python test script
- **bridge_node.py** - ROS 2 ↔ MQTT bridge (in my_mqtt_bridge/)

---

## 💡 Tips

1. **Always start the web server first** before opening HTML files
2. **Keep the server terminal open** while using the web UI
3. **Check the browser console** (F12) for detailed error messages
4. **Test connection** with mqtt_test.html before using main UI
5. **Verify ROS 2 bridge** is running if controlling actual robots

---

## 🎯 Typical Workflow

```bash
# Terminal 1: Start web server
cd /home/viswa/robot_ws/src/my_mqtt_bridge
python3 -m http.server 8000

# Terminal 2: Start ROS 2 (if needed)
ros2 launch my_mqtt_bridge fleet_with_mqtt_bridge.launch.py

# Browser: Open control interface
# http://localhost:8000/index.html
```

That's it! You're ready to control your robot fleet! 🚀
