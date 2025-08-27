# ROS 1 `cmd_vel` Bridge

This package provides a ROS 2 node to forward `cmd_vel` messages to a ROS 1 system via a WebSocket connection to a `rosbridge_server`.

## Core Functionality

- **Subscribes** to `/cmd_vel` on the ROS 2 network.
- **Forwards** messages to the `/cmd_vel` topic on a ROS 1 `rosbridge_server`.
- **On-Demand Forwarding:** Only forwards messages when they are received.
- **Safety Shutdown:** Sends a zero-velocity command upon exit to prevent runaway robots.
- **Parameterization:** The ROS 1 host and port are configurable.

## Dependency Management & Build Process

This package uses a modern Python packaging approach with a `pyproject.toml` file to manage its non-ROS dependencies (`roslibpy`, `pyyaml`, etc.) within a dedicated Python virtual environment (`.venv`). This is the key to making the package self-contained and reliable.

The installation and build process is as follows:

**1. Install Python Dependencies:**
First, the Python dependencies defined in `pyproject.toml` must be installed into the package's virtual environment. This is done using `pip` in "editable" mode (`-e`), which correctly links the package to the ROS 2 workspace while installing the necessary libraries.
```bash
# From the workspace root (e.g., ~/dev/robotics/shared_ros2)
source src/ros1_cmd_vel_bridge/.venv/bin/activate
pip install -e src/ros1_cmd_vel_bridge
```

**2. Build the ROS 2 Workspace:**
After the Python dependencies are correctly installed in the virtual environment, you can build the ROS 2 workspace as usual with `colcon`. `colcon` will create the necessary ROS 2 executables and launch files.
```bash
# From the workspace root (e.g., ~/dev/robotics/shared_ros2)
colcon build --symlink-install --packages-select ros1_cmd_vel_bridge
```
This two-step process ensures that when ROS 2 runs the node, the correct Python environment is used and all dependencies are found.

## Running the Node

There are three ways to run the node, from most to least convenient:

### 1. Using the `run_cmd_vel_forwarder` Alias (Recommended)

An alias is available in your `.zshrc` for maximum convenience. It handles sourcing all necessary environments.
```bash
# Run with default parameters
run_cmd_vel_forwarder

# Override the host IP
run_cmd_vel_forwarder ros1_host:=192.168.8.4
```

### 2. Using the `run.sh` Script

The `run.sh` script in the package directory also handles all environment sourcing.
```bash
# Run with default parameters
./src/ros1_cmd_vel_bridge/run.sh

# Override the host IP
./src/ros1_cmd_vel_bridge/run.sh ros1_host:=192.168.8.4
```

### 3. Using `ros2 launch` (Raw Command)

This method requires you to manually source both the ROS 2 workspace and the Python virtual environment first.
```bash
# Source environments
source /home/tony/dev/robotics/shared_ros2/install/setup.zsh
source /home/tony/dev/robotics/shared_ros2/src/ros1_cmd_vel_bridge/.venv/bin/activate

# Run the launch file
ros2 launch ros1_cmd_vel_bridge cmd_vel_forwarder.launch.py ros1_host:=192.168.8.4
```

## Parameters

*   `ros1_host` (string, default: "localhost"): The hostname or IP address of the ROS 1 machine running `rosbridge_server`.
*   `ros1_port` (integer, default: 9090): The port number of the `rosbridge_server`.