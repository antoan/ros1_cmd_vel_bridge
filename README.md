# ROS 1 `cmd_vel` Bridge

This package provides a ROS 2 node to forward `cmd_vel` messages to a ROS 1 system via a WebSocket connection to a `rosbridge_server`.

## Core Functionality

- **Subscribes** to `/cmd_vel` on the ROS 2 network.
- **Forwards** messages to the `/cmd_vel` topic on a ROS 1 `rosbridge_server`.
- **Resilient Connection:** The node will continuously attempt to reconnect if the connection is lost, with status updates logged.
- **Safety Shutdown:** Sends a zero-velocity command upon exit to prevent runaway robots.
- **Parameterization:** The ROS 1 host and port are configurable.

## Dependency Management & Build Process

This package uses a dedicated Python virtual environment (`.venv`) to manage its non-ROS dependencies (`roslibpy`, `pyyaml`, etc.). This makes the package self-contained and reliable.

The installation and build process is as follows:

**1. Install Python Dependencies:**
First, the Python dependencies must be installed into the package's virtual environment. This is done using `pip` in "editable" mode (`-e`), which correctly links the package to the ROS 2 workspace.
```bash
# From the workspace root (e.g., ~/dev/robotics/shared_ros2)
source src/ros1_cmd_vel_bridge/.venv/bin/activate
pip install -e src/ros1_cmd_vel_bridge
```

**2. Build the ROS 2 Workspace:**
After the Python dependencies are installed, build the ROS 2 workspace as usual with `colcon`.
```bash
# From the workspace root (e.g., ~/dev/robotics/shared_ros2)
colcon build --symlink-install --packages-select ros1_cmd_vel_bridge
```

## Running the Node

The recommended way to run the node is using `ros2 launch`, which now correctly handles the virtual environment.

### Using `ros2 launch` (Recommended)

The launch file is the most robust way to run the node. It has been specifically configured to use the Python interpreter from the package's virtual environment, which solves the `ModuleNotFoundError` for dependencies like `roslibpy`.

**How It Works:**
The `cmd_vel_forwarder.launch.py` file uses a `Node` action with a `prefix`. This `prefix` forces the node's executable to be run with the Python interpreter located inside the `.venv`. This ensures that all the correct dependencies are found and used, without needing to manually source the environment.

```bash
# Run with default parameters (connects to localhost:9090)
ros2 launch ros1_cmd_vel_bridge cmd_vel_forwarder.launch.py

# Override the host IP
ros2 launch ros1_cmd_vel_bridge cmd_vel_forwarder.launch.py ros1_host:=192.168.8.4
```

### Using the `run.sh` Script

The `run.sh` script in the package directory also handles all environment sourcing and is a convenient alternative.
```bash
# Run with default parameters
./src/ros1_cmd_vel_bridge/run.sh

# Override the host IP
./src/ros1_cmd_vel_bridge/run.sh ros1_host:=192.168.8.4
```

## Parameters

*   `ros1_host` (string, default: "localhost"): The hostname or IP address of the ROS 1 machine running `rosbridge_server`.
*   `ros1_port` (integer, default: 9090): The port number of the `rosbridge_server`.