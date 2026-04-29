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

After Python edits, rebuild: `colcon build --packages-select wall_robot_pkg`

## Architecture

Two comm paths from upper computer:
- **C++ WebSocket (agv_bridge)**: direct device handler dispatch -> /mission/command
- **Python ROSBridge (http_dispatcher)**: rosbridge -> /mission/params

Key topics: /mission/command (String), /cmd_vel_auto (String JSON), /mech/{m1,m2,ig35,push_rod}_target (Int32)

Key nodes: websocket_bridge_node(C++), mission_controller_node(Python), chassis_driver_node(Python), mechanism_driver_node(Python)

IG35: move_to_position(driver, target, speed_rpm, ...) in IG35.py. Speed=0x009A, Position=0x00D0.

Source: src/agv_protocol/ (C++ WebSocket lib), src/agv_bridge/ (C++ handlers), src/wall_robot_pkg/ (Python nodes)
