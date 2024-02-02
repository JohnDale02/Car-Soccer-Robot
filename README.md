# iRobot Car-Soccer: Hat-Trick Team Repository

## Overview

Welcome to the repository of the Hat-Trick intern team, dedicated to innovating the realm of robotic soccer with our custom-built robot. Centered around the iRobot Create3 and leveraging ROS2 for robust, real-time control, our project aims to push the boundaries of what's possible in robot soccer. The integration of an inexpensive Bluetooth controller allows for intuitive control over the robot's movement and shooting capabilities, making for an engaging and interactive experience.

https://github.com/JohnDale02/Car-Soccer-Robot/assets/116762794/2ba7e28f-fff7-4adf-98dc-dc0e13d0f43c

https://github.com/JohnDale02/Car-Soccer-Robot/assets/116762794/ba015392-aaaa-4fe9-990d-21689e540b33

## Key Features

- **iRobot Create3 Platform:** Utilizes the versatile and robust Create3 as the foundation for our soccer robot.
- **ROS2 Integration:** Employs Create3 platform's ROS2 support for efficient and reliable robot programming and control.
- **Bluetooth Control:** Enabling precise movement and shooting controls.

### Prerequisites

Ensure you have ROS2 installed on your system. For installation instructions, refer to the [official ROS2 documentation](https://docs.ros.org/en/foxy/Installation.html).

### Build and Run

To build and run the Create controller, follow these steps:

```bash
# Build
cd Car-Soccer-Robot/ros2_ws
colcon build
source install/setup.bash

# Run
ros2 run create_controller create_controller
```
