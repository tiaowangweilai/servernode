#!/usr/bin/env python3
"""motor_485 简短测试：M1下移->M2伸出->M2收回->M1上升，跑2次循环"""
import sys
sys.path.insert(0, "/home/c403/jiang/servernode_2026.4.28/src/wall_robot_pkg/wall_robot_pkg")
from motor_485 import MD2202Controller, scan_sequence
import time

motor = MD2202Controller(port='/dev/ttyACM0', baudrate=9600)
if motor.ser is None:
    print("FAIL: serial port")
    sys.exit(1)

print("--- init & reset ---")
motor.reset_m1(); time.sleep(2.0)
motor.reset_m2(); time.sleep(4.0)

for i in range(2):
    print(f"\n=== cycle {i+1}/2 ===")
    scan_sequence(motor, m1_pulse=200, m2_pulse=500)

motor.close()
print("\nDONE")
