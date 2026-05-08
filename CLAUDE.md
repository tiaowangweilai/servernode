# CLAUDE.md

This file provides guidance to Claude Code when working in this repository.

## Build & Run

```bash
source /opt/ros/humble/setup.bash
cd /home/c403/jiang/servernode_2026.4.28
colcon build
source install/setup.bash
ros2 launch wall_robot_pkg system_bringup.launch.py
```

After editing Python (wall_robot_pkg): `colcon build --packages-select wall_robot_pkg`

After editing C++: `colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Debug`

Run a single node: `ros2 run wall_robot_pkg <node_name>`

Test WebSocket client: `python3 src/startup/test_websocket_client.py`

Start the startup WebSocket server: `bash src/startup/start_agv_server.sh`

## Architecture

Two comm paths from upper computer:
1. **C++ WebSocket (agv_bridge, ports 9100→up/9001←local)**: DeviceHandler dispatch → `/mission/command`, `/cmd_vel_manual`
2. **Python ROSBridge (http_dispatcher, port 9090)**: rosbridge → `/web_to_dispatcher` → `/mission/params`

### Robot Types (C++ `robot_move` + `agv_bridge`)

| Type | ID | Launch |
|---|---|---|
| Mobile Dual-Arm | `mobile_dual_arm_robot` | `agv_bringup/agv.launch.xml` + `robot_go_target` |
| Vacuum Adsorption (wall) | `vacuum_adsorption_robot` | `wall_robot_pkg/system_bringup.launch.py` |

### ROS 2 Packages

- **agv_protocol** (C++): WebSocket client/server, JSON command parser (ASIO)
- **agv_bridge** (C++): WebSocket bridge node, polymorphic Robot + DeviceHandler (Chassis/Arms/Radar/Camera handlers)
- **robot_move** (C++): ROS node + factory pattern (agv/air/duct/mag)
- **wall_robot_pkg** (Python): All core robot nodes — mission control, chassis/mechanism drivers, vision, server, MJPEG
- **agv_bringup** (C++): AGV launch + controller config
- **agv_description** (C++): URDF, meshes
- **agv_moveit_config** (C++): MoveIt2 arm planning config
- **realsense-ros** (C++): Intel RealSense ROS driver
- **startup** (Python): Startup scripts
- **tools/** (Python): ws_video_server, ws_viewer

### Key Nodes (wall_robot_pkg)

| Node | File | Role |
|---|---|---|
| mission_controller_node | mission_controller_node.py | Mission state machine: PID navigation, waypoint tracking, path planning |
| chassis_driver_node | chassis_driver_node.py | Serial→PWM chassis (CH341, 115200). Subs `/cmd_vel_manual` (Twist) + `/cmd_vel_auto` (String JSON) |
| mechanism_driver_node | mechanism_driver_node.py | IG35, M1/M2 (MD2202), push rod via shared `/dev/ttyACM1` |
| http_dispatcher | http_dispatcher.py | rosbridge dispatcher (entry point "server_node") |
| vision_processor_node | vision_processor_node.py | Depth edge detection, safety zones |
| mjpeg_server_node | mjpeg_server_node.py | MJPEG stream on port 5000 |
| sick_odom_node | sick_odom_node.py | SICK LiDAR odometry |

### Key Topics

| Topic | Type | Purpose |
|---|---|---|
| `/mission/command` | String | System commands (single_scan) |
| `/mission/params` | String | JSON mission params from upper computer |
| `/cmd_vel_manual` | Twist | Manual velocity (takes priority over auto for 1s) |
| `/cmd_vel_auto` | String JSON | Auto navigation `{vx,vy,wz}` or nav path |
| `/mech/{m1,m2,ig35}_target` | Int32 | Mechanism targets |
| `/mech/push_rod_time` | Int32 | Push rod duration ms |
| `/chassis/serial_status` | String | "ALIVE" health check |

### C++ DeviceHandler Hierarchy (agv_bridge)

Base: `DeviceHandler` → `init(node)`, `handleCommand()`, `isOnline()`, `getReport()`

- **WallChassisHandler**: `/cmd_vel_manual` + `/mission/command` for `move`/`single_scan`
- **AgvChassisHandler**: `cmd_vel` (Float64MultiArray) + `/mission/sys_command`
- **ArmHandler**: `cmd_arm_joint` + `cmd_arm_cartesian` for dual-arm joint/cartesian moves
- **RadarHandler**: `/cmd_vel_auto` for nav path, heartbeat from `/odom`
- **WallCameraHandler / AgvCameraHandler**: `image_pos` click → `/click_point`

### Hardware

- **Chassis**: SBUS 16-ch PWM via `/dev/ttyCH341USB0` 115200 (range 1255-1745, mid 1500)
- **IG35**: CAN linear actuator via `/dev/ttyACM1` shared. `move_to_position(driver, target, speed_rpm, ...)`
- **M1/M2**: MD2202 steppers on same `/dev/ttyACM1`. `set_pos_m1(pulse)` / `set_pos_m2(pulse)`
- **Push rod**: GPIO. `push_rod_forward_time(duration_ms)`
- **Navigation**: UDP port 5010. PID: DIST_KP=10.0, YAW_KP=10.0

### Manual Priority

Chassis driver ignores `/cmd_vel_auto` if `/cmd_vel_manual` was received within the last 1 second.
