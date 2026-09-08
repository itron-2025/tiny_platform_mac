#!/usr/bin/env python3
"""Watch what teleop is commanding and what the wheels actually do.

Subscribes to /tiny/cmd_vel and /tiny/wheel_ticks and prints a line whenever the
command changes, with the wheel speeds measured over the same window. Use it to
confirm the gamepad is reaching the robot, and to see whether the wheels are
keeping up with what was asked.

Usage (inside the container, agent + teleop running):
    python3 tools/teleop_monitor.py --duration 120
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

WHEELS = ("FL", "FR", "RL", "RR")
COUNTS_PER_WHEEL_REV = 574.0
REPORT_HZ = 4.0


class Monitor(Node):
    def __init__(self):
        super().__init__("teleop_monitor")
        self.cmd = (0.0, 0.0, 0.0)
        self.n_cmd = 0
        self.ticks = None
        self.t_ticks = 0.0
        self._prev = None
        self._prev_t = 0.0
        self._last_report = 0.0
        self.peak = (0.0, 0.0, 0.0)
        self.create_subscription(Twist, "/tiny/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Int32MultiArray, "/tiny/wheel_ticks", self._on_ticks, 50)

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.n_cmd += 1
        self.peak = tuple(max(abs(c), p) for c, p in zip(self.cmd, self.peak))

    def _on_ticks(self, m):
        d = list(m.data)[:4]
        if len(d) != 4:
            return
        now = time.monotonic()
        self.ticks, self.t_ticks = d, now
        if self._prev is None:
            self._prev, self._prev_t = d, now
            return
        if now - self._last_report < 1.0 / REPORT_HZ:
            return
        dt = now - self._prev_t
        omega = [((d[i] - self._prev[i]) / COUNTS_PER_WHEEL_REV) * 2 * math.pi / dt
                 for i in range(4)]
        self._prev, self._prev_t, self._last_report = d, now, now

        vx, vy, wz = self.cmd
        moving = any(abs(w) > 0.5 for w in omega) or abs(vx) + abs(vy) + abs(wz) > 1e-3
        if not moving:
            return
        cells = "  ".join(f"{WHEELS[i]}{omega[i]:+6.1f}" for i in range(4))
        print(f"cmd vx{vx:+5.2f} vy{vy:+5.2f} wz{wz:+5.2f} | {cells}")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    node = Monitor()
    print("watching /tiny/cmd_vel and /tiny/wheel_ticks -- drive the robot\n")
    sys.stdout.flush()
    end = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    print(f"\n{node.n_cmd} /tiny/cmd_vel messages seen")
    if node.n_cmd == 0:
        print("  NOTHING ARRIVED. The deadman (button L) was never held, or")
        print("  joy_teleop is not running, or the controller dropped out.")
    else:
        print(f"  peak commanded: vx {node.peak[0]:.2f} m/s, "
              f"vy {node.peak[1]:.2f} m/s, wz {node.peak[2]:.2f} rad/s")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
