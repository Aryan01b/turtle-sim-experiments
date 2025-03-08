# Turtle Project

A ROS2 package for controlling the TurtleSim turtle with a simple movement pattern. This project demonstrates basic ROS2 concepts including publishers, subscribers, and timers.

## Overview

This package provides a simple turtle controller node that makes the turtle move in a circular pattern. It subscribes to the turtle's pose and publishes velocity commands.

## Dependencies

- ROS2 (tested on ROS2 Humble)
- Python 3.8+
- turtlesim package
- geometry_msgs package
- rclpy package

## Installation

### 1. Clone the repository into your ROS2 workspace

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/your-username/turtle_project.git
```

### 2. Install dependencies

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlesim
```

### 3. Build the package

```bash
cd ~/your_ros2_ws
colcon build --packages-select turtle_project
source install/setup.bash
```

## Usage

### 1. Launch TurtleSim

In one terminal, start the TurtleSim node:

```bash
ros2 run turtlesim turtlesim_node
```

### 2. Run the Turtle Controller

In another terminal, run the turtle controller node:

```bash
ros2 run turtle_project turtle_controller
```

You should see the turtle moving in a circular pattern, and position information being logged to the console.

## Node Details

### turtle_controller

#### Published Topics:
- `turtle1/cmd_vel` (geometry_msgs/msg/Twist): Commands to control the turtle's movement

#### Subscribed Topics:
- `turtle1/pose` (turtlesim/msg/Pose): The current position and orientation of the turtle

## Visualization

You can visualize the data from this package using:

### PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
```

### RQT Multiplot

```bash
ros2 run rqt_multiplot rqt_multiplot
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Authors

- Your Name <your.email@example.com>
