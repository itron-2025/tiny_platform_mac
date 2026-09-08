#!/usr/bin/env python3
"""Open-loop duty staircase: measures the deadband, the duty->speed line and the
top speed of all four wheels in one run. Bring-up stage E, plus the raw material
for the closed-loop feedforward.

CHASSIS MUST BE ELEVATED. This ends at full duty on all four wheels.

At each duty step it holds all four wheels steady, waits out the acceleration,
then measures speed from the tick delta over the settled part of the step (not
from /tiny/wheel_actual_vel, which is a single unfiltered 20 ms difference and
far too noisy to fit a line through).

What comes out:
  * deadband   -- the lowest duty that actually turns each wheel. Becomes
                  MIN_DUTY, and explains why a low /cmd_vel does nothing.
  * duty = offset + gain * omega, fitted over the moving region. These are
                  exactly FF_OFFSET / FF_GAIN for the closed-loop firmware, and
                  they are PER WHEEL on purpose: the spread between four
                  nominally identical motors is what makes an open-loop robot
                  curve instead of driving straight.
  * top speed  -- MAX_WHEEL_RAD_S is taken from the SLOWEST wheel, because the
                  inverse kinematics can only ask for a speed all four can
                  actually reach, then x0.8 for battery sag and load.

Usage (inside the container, agent running, chassis ELEVATED):
    python3 tools/duty_sweep.py
    python3 tools/duty_sweep.py --max-duty 600     # stop short of full
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

WHEELS = ("FL", "FR", "RL", "RR")
COUNTS_PER_WHEEL_REV = 574.0     # mirror of firmware config.h (measured)
PUBLISH_HZ = 20.0
STEPS = (0, 60, 80, 100, 120, 150, 200, 260, 330, 420, 520, 650, 800, 1023)
SETTLE_FRACTION = 0.45           # ignore this much of each step as acceleration


class Sweep(Node):
    def __init__(self):
        super().__init__("duty_sweep")
        self.pub = self.create_publisher(Float32MultiArray, "/tiny/wheel_duty", 10)
        self.ticks = None
        self.t_ticks = 0.0
        self.create_subscription(Int32MultiArray, "/tiny/wheel_ticks", self._cb, 50)

    def _cb(self, msg):
        d = list(msg.data)[:4]
        if len(d) == 4:
            self.ticks = d
            self.t_ticks = time.monotonic()

    def hold(self, duty, seconds):
        period = 1.0 / PUBLISH_HZ
        end = time.monotonic() + seconds
        m = Float32MultiArray()
        m.data = [float(duty)] * 4
        while rclpy.ok() and time.monotonic() < end:
            self.pub.publish(m)
            t = time.monotonic() + period
            while rclpy.ok() and time.monotonic() < t:
                rclpy.spin_once(self, timeout_sec=0.005)

    def stop(self):
        m = Float32MultiArray()
        m.data = [0.0] * 4
        for _ in range(5):
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ticks(self, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self.ticks is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ticks is not None


def fit_line(xs, ys):
    """Least squares y = a + b*x. Returns (a, b) or None if degenerate."""
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-9:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx
    return my - b * mx, b


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dwell", type=float, default=2.0, help="seconds per step")
    ap.add_argument("--max-duty", type=float, default=1023.0)
    ap.add_argument("--sign", type=int, choices=(1, -1), default=1,
                    help="+1 sweeps forward, -1 sweeps in reverse. Both are "
                         "needed: breakaway friction is NOT symmetric, and a "
                         "forward-only measurement leaves every reversing wheel "
                         "unable to start (measured on this chassis 2026-09-08).")
    args = ap.parse_args()

    steps = [d for d in STEPS if d <= args.max_duty]

    rclpy.init()
    node = Sweep()
    if not node.wait_ticks():
        print("No /tiny/wheel_ticks. Is the agent running?")
        return 1

    direction = "FORWARD" if args.sign > 0 else "REVERSE"
    print(f"CHASSIS MUST BE ELEVATED. Ramping {direction} duty 0 ->", steps[-1])
    print(f"{len(steps)} steps x {args.dwell:.1f}s\n")
    print(f"{'duty':>6}" + "".join(f"{w + ' rad/s':>12}" for w in WHEELS))
    print("-" * 56)
    sys.stdout.flush()

    table = []
    try:
        for duty in steps:
            # Accelerate, then measure only the settled tail of the step.
            node.hold(duty * args.sign, args.dwell * SETTLE_FRACTION)
            t0, c0 = node.t_ticks, list(node.ticks)
            node.hold(duty * args.sign, args.dwell * (1 - SETTLE_FRACTION))
            t1, c1 = node.t_ticks, list(node.ticks)

            dt = t1 - t0
            if dt <= 0:
                continue
            # Report magnitudes so a reverse sweep reads like a forward one.
            omega = [abs(((c1[i] - c0[i]) / COUNTS_PER_WHEEL_REV) * 2 * math.pi / dt)
                     for i in range(4)]
            table.append((duty, omega))
            print(f"{duty:>6}" + "".join(f"{w:>12.2f}" for w in omega), flush=True)
    finally:
        node.stop()

    print("\n" + "=" * 68)
    tops, results = [], []
    for i, name in enumerate(WHEELS):
        pts = [(d, o[i]) for d, o in table]
        moving = [(d, w) for d, w in pts if w > 1.0]
        dead = min((d for d, w in moving), default=None)
        top = max((w for _, w in pts), default=0.0)
        tops.append(top)
        # Fit duty = offset + gain*omega over the moving region only: below the
        # deadband the relation is not a line at all and would drag the fit.
        fit = fit_line([w for _, w in moving], [d for d, _ in moving]) if len(moving) > 2 else None
        results.append((name, dead, top, fit))
        f = f"duty = {fit[0]:6.1f} + {fit[1]:5.2f} * omega" if fit else "not enough points"
        print(f"  {name}:  deadband ~{dead if dead is not None else '?':>4}  "
              f"top {top:6.2f} rad/s   {f}")

    print("=" * 68)
    if tops and min(tops) > 0:
        slowest = min(tops)
        print(f"  slowest wheel tops out at {slowest:.2f} rad/s "
              f"(fastest {max(tops):.2f}, spread {(max(tops)/slowest - 1)*100:.1f}%)")
        print(f"  -> MAX_WHEEL_RAD_S = {slowest * 0.8:.1f}   (0.8 x slowest)")
        print(f"  -> that is vx max {slowest * 0.8 * 0.040:.2f} m/s at the 40 mm wheel")
        deads = [d for _, d, _, _ in results if d is not None]
        if deads:
            print(f"  -> MIN_DUTY = {max(deads):.0f}   (worst wheel's deadband)")
        print("\n  For the closed loop, FF_OFFSET / FF_GAIN are the fitted "
              "intercept / slope above,")
        print("  per wheel. Do not average them: the spread is the whole point.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
