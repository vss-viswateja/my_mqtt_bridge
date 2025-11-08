# Multi-Robot IoT Fleet Management System

## 1. Introduction

This project demonstrates a robust, scalable architecture for controlling a fleet of autonomous robots through a web-based interface. It integrates a high-fidelity multi-robot simulation with a cloud-native IoT messaging protocol (MQTT) to achieve real-time command and control.

The core of this system is a custom-built ROS 2 bridge that translates high-level navigation commands, published from a web browser, into executable action goals for individual robots. Each robot operates independently within the simulation, running its own instance of the ROS 2 Navigation Stack (Nav2).

This system is built upon the **`remroc` (Realistic Multi-Robot Coordination)** framework, which provides the foundational Gazebo simulation environment, detailed robot models, and pre-configured, namespaced Nav2 stacks. This project extends that framework by adding an IoT-based fleet management layer.

The data flow is as follows:
1.  **Web UI:** A user selects a robot and a target (X, Y) coordinate in a web browser.
2.  **MQTT Broker:** The browser publishes a JSON-formatted command to a central MQTT broker.
3.  **ROS 2 Bridge:** A custom ROS 2 node (`my_mqtt_bridge`) subscribes to the MQTT topic, parses the JSON, and identifies the target robot.
4.  **ROS 2 Actions:** The bridge node sends a formal `MapsToPose` action goal to the specified robot's namespaced Nav2 action server.
5.  **Simulation:** The robot in the Gazebo simulation executes the navigation task using its onboard Nav2 stack.



---

## 2. Prerequisites

Before launching the system, ensure all dependencies are met and all components are correctly installed and built.

### 2.1. Core Components

* **ROS 2 (Humble):** This project is developed and tested on ROS 2 Humble. A complete installation (e.g., `ros-humble-desktop-full`) is required.
* **Gazebo (Ignition Fortress):** The `remroc` framework depends on Gazebo Fortress, which is the default simulation environment for ROS 2 Humble.
* **Colcon:** The standard ROS 2 build tool.

### 2.2. Package Dependencies

* **`remroc` Repository:** The `remroc` repository is a critical dependency. It must be cloned into your ROS 2 workspace, and all its dependencies (including Nav2) must be installed.
    * **Installation:** Follow the installation guide from the official [boschresearch/remroc repository](https://github.com/boschresearch/remroc).
* **`paho-mqtt`:** This Python library is required for the ROS 2 bridge to communicate with the MQTT broker. It must be installed in your environment.
    ```bash
    pip install paho-mqtt
    ```
* **`my_mqtt_bridge` (This Package):** This package must be located in the same ROS 2 workspace as `remroc` and built successfully.
    ```bash
    # From the root of your workspace
    colcon build --packages-select my_mqtt_bridge
    ```

---

## 3. Instructions

The system is launched using a two-terminal process. One terminal initiates the core `remroc` simulation server, and the second terminal launches the `my_mqtt_bridge` node.

### 3.1. System Architecture

This decoupled launch process is by design:
* **Terminal 1 (Simulation Server):** This runs the "backend," which includes the main Gazebo Fortress server (`gz sim -s`), spawns all robots, and launches the individual, namespaced Nav2 stacks for each robot.
* **Terminal 2 (MQTT Bridge):** This runs the "translator" node. It connects the ROS 2 ecosystem to the external MQTT broker, listening for commands and dispatching them to the appropriate action servers running in Terminal 1.

### 3.2. Launching the System

#### Terminal 1: Launch the Simulation Server

This terminal will run the main simulation environment.

1.  Navigate to the root of your ROS 2 workspace and source the overlay:
    ```bash
    # (Assuming workspace root is ~/ros2_ws)
    cd ~/ros2_ws
    source install/setup.bash
    ```

2.  Execute the `start_server.sh` script. This script is a convenience wrapper that launches the `remroc` simulation environment. (Ensure this script is executable: `chmod +x start_server.sh`).
    ```bash
    cd src/my_mqtt_bridge
    ./start_server.sh
    ```

3.  Wait for the Gazebo environment to fully load. You should see log output indicating that the robot models are being spawned and the Nav2 stacks are becoming active. This process may take a few minutes.

#### Terminal 2: Launch the MQTT Bridge

This terminal will run the custom bridge node that listens for web commands.

1.  Open a **new terminal window**.

2.  Navigate to the root of your ROS 2 workspace and source the overlay:
    ```bash
    # (Assuming workspace root is ~/ros2_ws)
    cd ~/ros2_ws
    source install/setup.bash
    ```

3.  Use `ros2 launch` to start the `fleet_with_mqtt_bridge.launch.py` file. This launch file is configured to start the `bridge_node` from the `my_mqtt_bridge` package.
    ```bash
    ros2 launch my_mqtt_bridge fleet_with_mqtt_bridge.launch.py
    ```

4.  Observe the log output. You should see a confirmation message that the node has started and has successfully connected to the MQTT broker.

### 3.3. Sending Robot Commands

Once both terminals are running, the system is operational.

1.  Open the `index.html` file (provided in the `web_ui` directory of this package) in any modern web browser (e.g., Chrome, Firefox).

2.  The "Status" on the web page should change from "Connecting..." to "Connected," indicating it has successfully established a WebSocket connection with the MQTT broker.

3.  To send a command:
    * Select the target robot from the dropdown menu (e.g., `SmallDeliveryRobot_0`).
    * Enter the desired target `X Position` and `Y Position` coordinates (relative to the `map` frame in the simulation).
    * Click the **Send Goal** button.

4.  You can observe the **Terminal 2** window to see the `my_mqtt_bridge` node log that it has received the command. Subsequently, the corresponding robot in the **Gazebo** simulation will begin to execute its navigation task.
