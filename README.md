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

### Clock defaults and tutorial autograder

| Launch / configuration | `use_sim_time` default | Robot-description publisher |
| --- | --- | --- |
| `vehicle_sim.launch.py` | `true` | Provided through spawn |
| `localization.launch.py` | `true` | Disabled unless explicitly enabled |
| `controller.launch.py`, `direct_controller.launch.py` | `true` | None |
| `spawn.launch.py` | `true` | Enabled |
| EKF / NavSat YAML, nodes started directly with `ros2 run` | `false` | — |

Launch arguments apply only to the processes that launch starts. For perception or autonomy started separately with `ros2 run`, add `--ros-args -p use_sim_time:=true` in simulation. On hardware, use `false`. All nodes processing the vehicle's timed data should use the same clock; simulation time pauses with Gazebo and needs `/clock` messages.

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

## Simulation Quick start
```bash
ros2 launch stinger_bringup vehicle_sim.launch.py
```

## Multi-boat Simulation

Start the simulation and the first boat (`stinger`) in Terminal 1:

```bash
ros2 launch stinger_bringup vehicle_sim.launch.py
```

Spawn a second boat named `stinger2` three metres from the first in Terminal 2:

```bash
ros2 launch stinger_description spawn.launch.py \
  robot_name:=stinger2 prefix:=stinger2_ topic_prefix:=stinger2 \
  use_sim_time:=true x:=3.0 y:=0.0
```

Start the second boat's Gazebo bridge and localization in separate terminals:

```bash
# Terminal 3
ros2 launch stinger_sim vehicle_bridge.launch.py \
  model_name:=stinger2 topic_prefix:=stinger2 frame_prefix:=stinger2_
```

```bash
# Terminal 4
ros2 launch stinger_bringup localization.launch.py \
  robot_name:=stinger2 prefix:=stinger2_ use_sim_time:=true
```

Use a unique `robot_name`, `topic_prefix`, and `prefix` for each additional
boat, and set a distinct `x`/`y` spawn position. For each one, start its own
`spawn.launch.py`, `vehicle_bridge.launch.py`, and `localization.launch.py`
instances using those matching names and prefixes.

## Container Environment (Qix)

This stack uses [Qix](https://github.gatech.edu/ASDL-Robotics/qix) to manage reproducible ROS 2 Jazzy container environments.

### 1. Install & Enter the Container
From the root of `stinger-software`:
```bash
# Build and setup the container
qix stack install .

# Enter the container shell
qix stack enter stinger-software
```

### 2. Install ROS Dependencies (Inside Container)
To install binary dependencies for workspace packages:
```bash
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -y -r
colcon build
```

### 3. Recording Rosbags
Host storage is bind-mounted to `~/bags` inside the container:
```bash
# Records bag directly to ~/bags on your host machine
ros2 bag record -o ~/bags/<bag_name> <topics>
```

## Multi-Vehicle Simulation (Lyoko)

To run multi-boat experiments in simulation using [Lyoko](https://github.gatech.edu/ASDL-Robotics/lyoko):

```bash
# Launch two simulated stingers in a shared Gazebo world
ros2 launch lyoko_gz_bringup spawn_experiment.launch.py experiment_name:=two_stingers
```

This starts the simulation world, computes staggered spawn poses, spawns each boat's URDF, configures per-vehicle topic bridges, and launches bringup stacks under `/stinger_1` and `/stinger_2`.
