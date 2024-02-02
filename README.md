# iRobot Car-Soccer: Hat-Trick Team Repository

## Overview

Welcome to the Hat-Trick intern team's repository, where innovation meets competition in our quest to excel in the intern challenge with our specially designed robot. At the heart of our project lies the iRobot Create3, enhanced with ROS2 for superior real-time control. Our design draws inspiration from the dynamic action of pinball machines, featuring individually controlled left and right flippers/paddles for unparalleled offensive and defensive maneuvers. (UPDATE: WE LOST!!; Congrats to the other Intern team and their robot. PS. Paul if you're reading this, you got lucky)

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
