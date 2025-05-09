# stinger-software
Stinger Tug Core Software

## Overview
![architecture](resources/architecture.excalidraw.svg)

### A Closer look at the Overall structure

```
stinger_bringup
    drivers for imu, gps, camera, lidar
    launch file for publishing odom using robot localization
    launch file for publishing static tf
    node for controlling the motors
    launch file for simulation
stinger_description
    urdf  of the boat
stinger_autonomy
    detection.py
    navigate.py
    state_manager.py
stinger_sim
    simulation world definition
    hooks and bridges for sensor emulation
```

## Quick Start 
Follow INSTALL.md to install all the requirements

Bringup nodes:
```
    ros2 launch stinger_bringup sensors.launch.py
        If wish to run independently:
        - ros2 launch sllidar_ros2 sllidar_c1_launch.py
        - ros2 run stinger_bringup camera-node
    ros2 launch stinger_bringup ekf.launch.py
    ros2 launch stinger_bringup tf.launch.py
    ros2 run stinger_bringup motor-node
        - ros2 topic pub /thrusters/left/thrust std_msgs/msg/Float64 "data: 20.0"
```
Autonomy nodes:
```
    ros2 run stinger_autonomy detection-node
    ros2 run stinger_autonomy controller-node
```

## Simulation Quick start
```
ros2 launch stinger_bringup vehicle_sim.launch.py
```