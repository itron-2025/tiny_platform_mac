#!/usr/bin/env python3
"""Stage F: verify the /tiny/cmd_vel -> inverse kinematics -> feedforward chain.

Commands each body axis in turn and compares the wheel speeds the mecanum
inverse kinematics ASKED for against the speeds the encoders actually saw. That
checks three things at once: the sign pattern of the kinematics, the accuracy of
the open-loop duty map, and whether any wheel is failing to keep up.

    +vx  forward      expected wheel signs  + + + +
    +vy  strafe LEFT                        - + + -
    +wz  yaw CCW                            - + - +

WHAT THIS STILL CANNOT TELL YOU. It compares the firmware against itself: the
expected speeds come from the same formula the firmware runs, so a matching sign
pattern proves the map is being applied, NOT that the robot moves the way the
command says. Whether +vy actually slides the chassis LEFT depends on the
mecanum rollers forming an X when seen from above, which is a mechanical fact no
encoder can see. Watch the robot (or put it on the floor) for that one.

CHASSIS ELEVATED for the first run.

Usage:
    python3 tools/cmd_vel_check.py
    python3 tools/cmd_vel_check.py --vx 0.2 --vy 0.2 --wz 0.8   # gentler
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
COUNTS_PER_WHEEL_REV = 574.0     # mirror of firmware config.h
WHEEL_RADIUS_M = 0.040
L_SUM = 0.102 + 0.1625
PUBLISH_HZ = 20.0
SETTLE_FRACTION = 0.45


def inverse_kinematics(vx, vy, wz):
    r = WHEEL_RADIUS_M
    return [
        (vx - vy - L_SUM * wz) / r,   # FL
        (vx + vy + L_SUM * wz) / r,   # FR
        (vx + vy - L_SUM * wz) / r,   # RL
        (vx - vy + L_SUM * wz) / r,   # RR
    ]


class Check(Node):
    def __init__(self):
        super().__init__("cmd_vel_check")
        self.pub = self.create_publisher(Twist, "/tiny/cmd_vel", 10)
        self.ticks = None
        self.t_ticks = 0.0
        self.create_subscription(Int32MultiArray, "/tiny/wheel_ticks", self._cb, 50)

    def _cb(self, msg):
        d = list(msg.data)[:4]
        if len(d) == 4:
            self.ticks, self.t_ticks = d, time.monotonic()

    def hold(self, vx, vy, wz, seconds):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        period = 1.0 / PUBLISH_HZ
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.pub.publish(m)
            t = time.monotonic() + period
            while rclpy.ok() and time.monotonic() < t:
                rclpy.spin_once(self, timeout_sec=0.005)

    def wait_ticks(self, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self.ticks is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ticks is not None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vx", type=float, default=0.4, help="m/s for the forward test")
    ap.add_argument("--vy", type=float, default=0.4, help="m/s for the strafe test")
    ap.add_argument("--wz", type=float, default=1.5, help="rad/s for the yaw test")
    ap.add_argument("--dwell", type=float, default=3.0)
    args = ap.parse_args()

    cases = [
        ("forward  +vx", args.vx, 0.0, 0.0),
        ("strafe L +vy", 0.0, args.vy, 0.0),
        ("yaw CCW  +wz", 0.0, 0.0, args.wz),
    ]

    rclpy.init()
    node = Check()
    if not node.wait_ticks():
        print("No /tiny/wheel_ticks. Is the agent running?")
        return 1

    print("CHASSIS ELEVATED. Three axes, "
          f"{args.dwell:.1f}s each.\n")
    ok_all = True
    try:
        for label, vx, vy, wz in cases:
            want = inverse_kinematics(vx, vy, wz)

            node.hold(vx, vy, wz, args.dwell * SETTLE_FRACTION)
            t0, c0 = node.t_ticks, list(node.ticks)
            node.hold(vx, vy, wz, args.dwell * (1 - SETTLE_FRACTION))
            t1, c1 = node.t_ticks, list(node.ticks)
            node.hold(0, 0, 0, 1.2)

            dt = t1 - t0
            got = [((c1[i] - c0[i]) / COUNTS_PER_WHEEL_REV) * 2 * math.pi / dt
                   for i in range(4)]

            print(f"{label}   (vx={vx:.2f} vy={vy:.2f} wz={wz:.2f})")
            print(f"    {'':<6}{'want':>9}{'got':>9}{'err':>9}")
            for i, name in enumerate(WHEELS):
                err = (got[i] - want[i]) / abs(want[i]) * 100 if abs(want[i]) > 0.1 else 0.0
                flag = ""
                if want[i] * got[i] < 0:
                    flag = "  <-- WRONG SIGN"
                    ok_all = False
                elif abs(err) > 20:
                    flag = "  <-- off by >20%"
                    ok_all = False
                print(f"    {name:<6}{want[i]:>9.2f}{got[i]:>9.2f}{err:>8.1f}%{flag}")
            print()
            sys.stdout.flush()
    finally:
        m = Twist()
        for _ in range(5):
            node.pub.publish(m)
            time.sleep(0.02)

    print("=" * 60)
    if ok_all:
        print("  All three axes: signs correct, magnitudes within 20%.")
        print("  The /cmd_vel -> IK -> feedforward -> motor chain is consistent.")
    else:
        print("  Something above did not match -- see the flags.")
    print("  STILL UNVERIFIED, and encoders cannot see it: whether +vy actually")
    print("  slides the chassis LEFT. That depends on the mecanum rollers forming")
    print("  an X viewed from above. Watch the wheels, or drive it on the floor.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
