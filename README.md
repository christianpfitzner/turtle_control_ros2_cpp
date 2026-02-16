# turtle_control_ros2_cpp

A ROS2 C++ package for controlling turtles in the turtlesim simulator. This package demonstrates basic ROS2 concepts including service calls, publishers, and timer callbacks.

## Overview

This package provides a ROS2 node that spawns a second turtle in turtlesim and controls its movement. It demonstrates:
- Service client usage (spawning a turtle)
- Publisher creation for velocity commands
- Timer-based callback execution

## Prerequisites

- **ROS2** (tested with ROS2 Humble or later)
- **C++17** compatible compiler
- **turtlesim** package

## Dependencies

This package depends on the following ROS2 packages:
- `rclcpp` - ROS2 C++ client library
- `geometry_msgs` - Common geometric messages
- `std_msgs` - Standard message definitions
- `sensor_msgs` - Sensor message definitions
- `turtlesim` - Turtle simulator package

## Installation

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/christianpfitzner/turtle_control_ros2_cpp.git
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Building

Build the package using colcon:
```bash
cd ~/ros2_ws
colcon build --packages-select turtle_control_ros2_cpp
```

Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Start turtlesim in a separate terminal:
```bash
ros2 run turtlesim turtlesim_node
```

2. Run the turtle control node:
```bash
ros2 run turtle_control_ros2_cpp turtle_control_node
```

The node will:
- Spawn a second turtle named "turtle2" in the turtlesim window
- Publish velocity commands to control turtle2's movement

## Project Structure

```
turtle_control_ros2_cpp/
├── CMakeLists.txt          # CMake build configuration
├── package.xml             # ROS2 package manifest
├── README.md               # This file
└── src/
    └── control_node.cpp    # Main control node implementation
```

## Node Details

### turtle_control_node

**Published Topics:**
- `/turtle2/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands for turtle2

**Service Clients:**
- `/spawn` (`turtlesim/srv/Spawn`) - Used to spawn a new turtle

## Development

The main node (`TurtleControlNode`) includes placeholders for:
1. Setting spawn position parameters for the turtle
2. Implementing velocity control logic in the `publish_velocity()` method

## Testing

Run linting tests:
```bash
cd ~/ros2_ws
colcon test --packages-select turtle_control_ros2_cpp
colcon test-result --verbose
```

## License

TODO: License declaration

## Maintainer


- **Prof. Dr. Christian Pfitzner** - christian.pfitzner@th-nuernberg.de

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.
