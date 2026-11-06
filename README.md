# stinger-software
Stinger Tug Core Software

## Overview
![architecture](resources/stinger_architecture.svg)

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
    state manager
stinger_controller
    velocity controller
stinger_perception
    detect the gate
stinger_sim
    simulation world definition
    hooks and bridges for sensor emulation
```

## ROS tutorial / simulation quick start

The launch defaults prioritize ROS_Tutorial. Complete each student TODO when the tutorial reaches it; the merge keeps those exercises intact.

```bash
ros2 launch stinger_bringup vehicle_sim.launch.py
```

For Topic 4.4, start localization in a second terminal with the existing tutorial command:

```bash
ros2 launch stinger_bringup localization.launch.py
```

Both launches use simulation time by default. Localization leaves robot-description publishing disabled because `vehicle_sim.launch.py` already starts it through `spawn.launch.py`.

At the Topic 4.5 checkpoint, uncomment the localization block in `vehicle_sim.launch.py` as instructed. It forwards the simulation clock and disables its own robot-description publisher. Then stop the separately launched localization process and use only `vehicle_sim.launch.py`.

When needed, launch either controller in a separate terminal; both default to simulation time:

```bash
ros2 launch stinger_controller controller.launch.py
# Alternative: direct control without the velocity controller
ros2 launch stinger_controller direct_controller.launch.py
```

Run one controller launch at a time. The velocity controller still requires the student TODOs to be completed.

### Clock defaults and tutorial autograder

| Launch / configuration | `use_sim_time` default | Robot-description publisher |
| --- | --- | --- |
| `vehicle_sim.launch.py` | `true` | Provided through spawn |
| `localization.launch.py` | `true` | Disabled unless explicitly enabled |
| `controller.launch.py`, `direct_controller.launch.py` | `true` | None |
| `spawn.launch.py` | `true` | Enabled |
| EKF / NavSat YAML, nodes started directly with `ros2 run` | `false` | — |

Launch arguments apply only to the processes that launch starts. For perception or autonomy started separately with `ros2 run`, add `--ros-args -p use_sim_time:=true` in simulation. On hardware, use `false`. All nodes processing the vehicle's timed data should use the same clock; simulation time pauses with Gazebo and needs `/clock` messages.

Topic 4.4's autograder starts localization without Gazebo. Its `ROS_Tutorial/autograder/launch/test_4_4.launch.py` include must explicitly pass `use_sim_time:=false` and `publish_robot_description:=false`. Apply the companion tutorial change before using this merge with that grader; the student command remains `ros2 launch autograder test_4_4.launch.py`. NavSat remains the second `Node` definition for the Topic 4.4.b instructions.

To check a running node, use `ros2 param get /NODE_NAME use_sim_time`; in simulation, check the clock with `ros2 topic echo /clock --once`. Full localization, perception, and autonomy validation requires the corresponding student exercises.

## Hardware bringup (current manual procedure)
Follow INSTALL.md to install the requirements. These are the existing boat-specific instructions; the combined `vehicle_real.launch.py` and reproducible setup guide are being developed separately.

Connect to wifi on your laptop: `GL-MT3000-0a9`  OR  `GL-MT3000-0a9-5G`
  - Password: `boats0519`

For Research Demo Tug:
```
ssh tug1@192.168.8.189
pw: boats0519
```

Before bringup, grant necessary permissions in the terminal that you are going to launch the sensor:
```
    sudo pigpiod # for motors
    sudo chmod 666 /dev/serial0 # for gps
    sudo chmod 666 /dev/video0 # for camera
```

Bringup nodes:
```
    ros2 launch stinger_bringup sensors.launch.py
        If wish to run independently:
        - ros2 launch sllidar_ros2 sllidar_c1_launch.py
        - ros2 run stinger_bringup camera-node
    ros2 launch stinger_bringup localization.launch.py use_sim_time:=false publish_robot_description:=true
    ros2 run stinger_bringup motor-node
```
Autonomy nodes:
```
    ros2 run stinger_autonomy state_node
    ros2 run stinger_perception perception_node
    ros2 launch stinger_controller controller.launch.py use_sim_time:=false
```

The motor node consumes `/thrusters/left/thrust` and `/thrusters/right/thrust` as values in `[-100, 100]`. The controller launch publishes `/stinger/thruster_port/cmd_thrust` and `/stinger/thruster_stbd/cmd_thrust`; connecting these to the hardware motor interface requires agreed routing and units in the hardware bringup work.

The planned `vehicle_real.launch.py` must default `use_sim_time` to `false` and explicitly forward it to every included launch and node that accepts it. If localization supplies the robot-state publisher, pass `publish_robot_description:=true`; if the hardware wrapper supplies one, pass `false`. Run exactly one publisher. Keep shared YAML clock defaults at `false`, with launch overrides after the YAML parameters.
