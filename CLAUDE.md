# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ROS 2 Distro

Humble (not Jazzy -- the startup README is incorrect).

## Build & Run

```bash
source /opt/ros/humble/setup.bash
cd /home/c403/jiang/servernode_2026.4.28
colcon build
source install/setup.bash
ros2 launch wall_robot_pkg system_bringup.launch.py   # wall robot
ros2 launch agv_bringup agv.launch.xml                # mobile dual-arm
```

Partial build: `colcon build --packages-select <pkg>`
C++ debug: `colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Debug`
Run single node: `ros2 run wall_robot_pkg <node_name>`

## Robot Types

| Type | ID | Launch |
|---|---|---|
| Mobile Dual-Arm | `mobile_dual_arm_robot` | `agv_bringup/agv.launch.xml` + `robot_go_target` |
| Vacuum Adsorption (wall) | `vacuum_adsorption_robot` | `wall_robot_pkg/system_bringup.launch.py` |

## ROS 2 Packages

- **agv_protocol** (C++): WebSocket client/server, JSON command parser (ASIO)
- **agv_bridge** (C++): WebSocket bridge, polymorphic Robot + DeviceHandler dispatch
- **robot_move** (C++): ROS node + factory pattern (agv/air/duct/mag)
- **wall_robot_pkg** (Python): wall robot core nodes
- **agv_bringup** (C++): AGV launch + controller manager
- **agv_description** (C++): URDF, meshes
- **agv_moveit_config** (C++): MoveIt2 arm planning config
- **realsense-ros** (C++): Intel RealSense ROS driver
- **startup** (Python): AGV startup WS server (port 9001)

## Comm Paths

1. **C++ WS (agv_bridge):** listen 9100 <- upper, pub 9001 -> upper. DeviceHandler dispatch -> `/mission/command`, `/cmd_vel_manual`
2. **ROSBridge WS (http_dispatcher):** port 9090. `/web_to_dispatcher` -> `/mission/params`

## Key Nodes (wall_robot_pkg)

| Node | File | Role |
|---|---|---|
| mission_controller_node | mission_controller_node.py | PID nav, waypoint tracking, path planning |
| chassis_driver_node | chassis_driver_node.py | Serial->PWM chassis (CH341, 115200) |
| mechanism_driver_node | mechanism_driver_node.py | IG35, M1/M2 (MD2202), push rod via /dev/ttyACM1 |
| http_dispatcher (entry: server_node) | http_dispatcher.py | rosbridge dispatcher |
| vision_processor_node | vision_processor_node.py | Depth edge detection, safety zones |
| mjpeg_server_node | mjpeg_server_node.py | MJPEG on port 5000 |
| sick_odom_node | sick_odom_node.py | SICK LiDAR odometry |
| image_resizer_node | image_resizer.py | Image compression |

## Key Topics

| Topic | Type | Purpose |
|---|---|---|
| `/mission/command` | String | System commands |
| `/mission/sys_command` | String | System cmds from mission_controller |
| `/mission/params` | String | JSON mission params from upper |
| `/mission/state` | String | Mission state |
| `/mission/nav_goal` | Point | Nav goal point |
| `/mission/motor_cmd` | Point | Motor command |
| `/mission/target_idx` | Int32 | Waypoint index |
| `/mission/planned_path` | Path | Planned nav path |
| `/mission/click` | Point | Image click (camera->lidar) |
| `/mission/events` | String | Event cmds (capture/save/work_complete) |
| `/cmd_vel_manual` | Twist | Manual velocity (priority 1s) |
| `/cmd_vel_auto` | String | Auto nav {vx,vy,wz} |
| `/web_to_dispatcher` | String | Raw JSON from rosbridge (9090) |
| `/mech/m1_target` | Int32 | M1 stepper pulses |
| `/mech/m2_target` | Int32 | M2 stepper pulses |
| `/mech/ig35_target` | Int32 | IG35 position |
| `/mech/ig35_speed` | Int32 | IG35 speed |
| `/mech/push_rod_time` | Int32 | Push rod ms |
| `/chassis/serial_status` | String | ALIVE health |
| `/odom` | Odometry | SICK LiDAR odom |
| `/vision/edge_preview` | Image | Edge detection |
| `camera_status` | String | Camera heartbeat |

## DeviceHandler Hierarchy

Base: `DeviceHandler` -> `init(node)`, `handleCommand()`, `isOnline()`, `getReport()`

- **WallChassisHandler**: `/cmd_vel_manual` + `/mission/command` (move/single_scan)
- **AgvChassisHandler**: `cmd_vel` (Float64MultiArray) + `s/mission/sy_command`
- **ArmHandler**: `cmd_arm_joint` + `cmd_arm_cartesian`
- **RadarHandler**: `/cmd_vel_auto` nav path, `/odom` heartbeat
- **WallCameraHandler / AgvCameraHandler**: `image_pos` click -> `/click_point`

## Hardware

- **Chassis**: SBUS 16-ch PWM `/dev/ttyCH341USB0` 115200 (1255-1745, mid 1500)
- **IG35**: CAN linear actuator `/dev/ttyACM1`
- **M1/M2**: MD2202 steppers `/dev/ttyACM1`
- **Push rod**: GPIO
- **Nav**: UDP 5010, PID DIST_KP=10.0 YAW_KP=10.0
- **Camera**: RealSense D405 640x480@30 depth aligned to color

Manual priority: chassis ignores `/cmd_vel_auto` if `/cmd_vel_manual` rcvd <1s ago.

## Debugging

```bash
# Monitor topics
ros2 topic list
ros2 topic echo /mission/state
ros2 topic echo /chassis/serial_status

# Check logs
tail -f log/*.log

# Test tools
python3 src/startup/test_websocket_client.py
python3 test_events.py          # 1=capture, 2=save, 3=work_complete, q=quit
bash src/startup/start_agv_server.sh

# Video tools
python3 tools/ws_viewer.py          # on upper 192.168.137.134
python3 tools/ws_video_server.py    # on robot

# Diagnostics
cat nav_path_sent.json
ls -l /dev/ttyCH341USB0 /dev/ttyACM0 /dev/ttyACM1
```

## Startup WS Protocol (port 9001)

```json
{"target": "arm_left_start", "command": "true"}   # start
{"target": "arm_left_start", "command": "false"}  # stop
```
