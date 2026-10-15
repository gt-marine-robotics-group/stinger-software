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

## Standalone Multi-boat Simulation (without Lyoko)

Use this flow when launching Stinger's own Gazebo world. Do not use it together
with the Lyoko experiment below: `vehicle_sim.launch.py` starts its own world,
boat, and bridge.

Start the simulation and first boat (`stinger`) in Terminal 1:

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

## Two-Stinger Experiment (Lyoko)

Run this from the ROS 2 Jazzy container after dependencies are installed and the
workspace is built:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch lyoko_gz_bringup spawn_experiment.launch.py experiment_name:=two_stingers
```

Lyoko also imports the `urdf_bridge_builder` Python package at launch time. If
that import is missing, install the helper in a workspace-local virtual
environment and expose it to ROS's Python process:

```bash
cd ~/colcon_ws
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install "git+https://github.com/seantfish/urdf-bridge-builder.git"
export PYTHONPATH="$("$PWD/.venv/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')${PYTHONPATH:+:$PYTHONPATH}"
```

Run the export command again in any new terminal that launches Lyoko.

The `two_stingers` experiment config starts one Lyoko-owned world and clock,
spawns two Stingers at separate poses, creates each boat's Gazebo/ROS bridges,
and starts the Stinger per-boat adapter. Each adapter starts that boat's
robot-state publisher and localization stack using simulation time. Lyoko also
sets `sitl: none` for both boats, so ArduPilot is not started.

The first launch in a separate terminal owns the world and both boats. Do not
also launch `vehicle_sim.launch.py`, `spawn.launch.py`,
`vehicle_bridge.launch.py`, or `localization.launch.py` for either boat: those
would duplicate the world, model, bridges, or nodes already started by the
experiment.


### Launching optional nodes independently

The experiment already starts the world, boats, bridges, robot-state publishers,
and localization nodes. To avoid duplicates, launch only additional nodes you
need and set simulation time so they follow Gazebo's clock. For example, the
controller launch can be started separately:

```bash
ros2 launch stinger_controller controller.launch.py use_sim_time:=true
```

For a node started with `ros2 run`, pass the simulation-time parameter like
this:

```bash
ros2 run <package> <executable> --ros-args -p use_sim_time:=true
```

The current controller, perception, and autonomy nodes use legacy absolute
single-boat topics such as `/stinger/...` and `/cmd_vel`. They are not yet
namespaced for `/stinger_1` or `/stinger_2`; launching them does not make them
work with both boats. Do not start `sensors.launch.py` for this simulation,
since it starts hardware sensor drivers rather than using the experiment's
Gazebo bridges. For standalone Stinger simulation, use the standalone launch
instructions above and start only any nodes that `vehicle_sim.launch.py` does
not already provide.
