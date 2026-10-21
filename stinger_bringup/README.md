# stinger_bringup

## tug05 startup procedure (one-command launch)

Where each command runs:
- Laptop = your computer
- Boat = tug05, after you ssh in

The IP, username, and passwords are for tug05.

Each `ros2` command keeps its terminal busy, so open a new terminal tab and log in to the boat again for the next one.

### 1. Power on (Boat)
On the bench: turn on the 13 V wall supply first, then the boat. Wait about a minute for the Pi to boot.

### 2. Log in (Terminal)
Connect to the boat's Wi-Fi (GL-MT3000-0a9):
```
ssh tug05@192.168.2.205
```
Connect remotely with Tailscale (see "Remote access with Tailscale" below):
```
ssh tug05@100.99.163.64
```

### Update the code (SKIP if up to date)
```
cd ~/ors_ws/src/stinger-software
git fetch origin
git checkout hardware-launch
git pull
cd ~/ors_ws && colcon build
source install/setup.bash
```

### 3. Permissions (Once after every power-on)
```
sudo pigpiod; sudo chmod 666 /dev/serial0 /dev/video0
```

### 4. Start sensors, localization, and motors (terminal 1)
```
ros2 launch stinger_bringup hardware.launch.py
```
Starting with motors off:
```
ros2 launch stinger_bringup hardware.launch.py motors:=false
```

### 5. Drive with the keyboard (terminal 2)
Needs step 4 running with motors on.
```
ros2 run stinger_bringup teleop-keyboard
```

### 6. Checking data (separate terminal)
Run one at a time. Press Ctrl+C to stop.

| Sensor | Command | Expected |
|---|---|---|
| LiDAR | `ros2 topic hz /scan` | about 10 Hz |
| Camera | `ros2 topic hz /stinger/camera_0/image_raw` | about 9 Hz |
| IMU | `ros2 topic hz /stinger/imu/data` | about 10 Hz |
| GPS | `ros2 topic echo /stinger/gps/fix` | messages outdoors only |
| Odometry | `ros2 topic hz /stinger/odometry/filtered` | about 10 Hz |

### 7. Foxglove (optional)
Boat, new terminal:
```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Laptop: open the Foxglove desktop app, choose Open connection, then Foxglove WebSocket, and enter:
- Lab Wi-Fi: `ws://192.168.2.205:8765`
- Tailscale: `ws://100.99.163.64:8765`

### 8. Stop Boat
Press Ctrl+C in each terminal. If a thruster keeps spinning:
```
sudo pigs s 12 0 s 13 0
```

### 9. Shut down Boat
```
sudo shutdown now
```
Wait until the Pi's green light stops blinking, then turn off the boat, then the wall supply.


## Remote access with Tailscale

### One-time laptop setup
1. Install Tailscale: https://tailscale.com/download
2. Sign in.
3. Ask the Tailscale admin (currently: Edwin Wu) to add you.
4. In the Tailscale menu, check that `tug05-desktop` appears with address `100.99.163.64`.

### Every time
1. Make sure Tailscale is connected on your laptop.
2. `ssh tug05@100.99.163.64`
3. Continue from step 3 (Permissions) above.

### Troubleshooting
- ssh hangs: check that Tailscale is connected and `tug05-desktop` shows as online.
- Foxglove will not connect: check that `foxglove_bridge` is running on the boat (step 7).
