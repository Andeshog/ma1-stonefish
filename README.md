## A simulation of milliAmpere 1 in Stonefish
This package contains a simulation environment for the milliAmpere 1 ferry using Stonefish and ROS 2.

![mA1 in a canal environment](https://drive.google.com/uc?export=view&id=1279v2Hyj8uTmQvKLOYL-7Lph7UwX72m9)

## Prerequisites
- The [Stonefish](https://github.com/patrykcieslak/stonefish) library needs to be installed.
- The [Stonefish ROS 2 package](https://github.com/patrykcieslak/stonefish_ros2) must be compiled.

## Launching
Simulator
```bash
ros2 launch ma1_sim simulation.launch.py
```
Simulator with lower quality
```bash
ros2 launch ma1_sim simulation.launch.py rendering_quality:=low
```
Simulator without rendering
```bash
ros2 launch ma1_sim simulation_nogpu.launch.py
```
Thrust allocation
```bash
ros2 launch ma1_thrust_allocation ma1_thrust_allocation.launch.py
```
Joystick interface (requires a connected joystick)
```bash
ros2 launch ma1_joystick_interface ma1_joystick_interface.launch.py
```

## Simulated sensors
| Sensor | Topic | Message type |
|----------|----------|----------|
| GPS | /ma1/gps | [NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html) |
| IMU | /ma1/imu | [Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) |
| Odometry | /ma1/odom | [Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) |

## Simulated actuators
| Actuator | Topic | Message type |
|----------|----------|----------|
| Thrusters | /ma1/thrusters | [Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html) |
| Servo | /ma1/servos | [JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) |

The control input $\tau$ is published on topic `/ma1/tau` with type [Wrench](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html).
