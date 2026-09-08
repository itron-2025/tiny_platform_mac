#!/usr/bin/env python3
"""Release the CH340/CH341 RTS line so the ESP32's EN(reset) pin is not held low.

On this board RTS is wired to EN. When the micro-ROS agent opens the serial port
the kernel asserts RTS, which holds the ESP32 in reset forever -> the firmware
never boots -> no micro-ROS session -> empty `ros2 node list`. Run this AFTER the
agent has opened the port to drop RTS/DTR and let the board boot and connect.

Baked into the image at /root/scripts/release_rts.py and called by start-agent.sh.
If the RTS->EN auto-reset is later disabled on the board in hardware, this becomes
a harmless no-op.

Usage:  release_rts.py [DEV] [BAUD]
"""
import sys
import time

import serial

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/esp32"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

p = serial.Serial(DEV, BAUD)
p.setDTR(False)  # GPIO0 high -> normal boot, not download mode
p.setRTS(False)  # release EN -> board boots
time.sleep(0.2)
p.close()
print(f"RTS/DTR released on {DEV} -> ESP32 booting")
