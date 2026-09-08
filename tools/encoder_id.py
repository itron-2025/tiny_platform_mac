#!/usr/bin/env python3
"""Identify which encoder channel belongs to which physical wheel, and which way
each one counts -- without relying on anyone remembering what they did.

THE PROBLEM THIS SOLVES. The obvious test ("turn the wheels in order FL, FR, RL,
RR and I will watch the columns") depends on the order being followed exactly.
Walking around a chassis, FL -> FR -> RR -> RL is the natural loop, and the two
orders produce IDENTICAL data with opposite conclusions: either the wiring is
fine, or the two rear connectors are swapped. Asking afterwards does not help
much -- by then nobody is sure.

THE FIX. Turn each wheel a DIFFERENT NUMBER OF TIMES, with a pause between each:

    front-left  FL : 1 burst
    front-right FR : 2 bursts
    rear-left   RL : 3 bursts
    rear-right  RR : 4 bursts

The burst count is the wheel's identity, so the order does not matter and
neither does memory. The COLUMN the bursts land in is the ESP32 pin group. Put
those together and the mapping is unambiguous.

ROLL EVERY WHEEL IN THE SAME SENSE: the direction that would drive the robot
FORWARD (the top of the wheel travels toward the front). The sign of each column
is then exactly ENC_DIR for that wheel.

Usage (inside the container, agent running):
    python3 tools/encoder_id.py --duration 180

Nothing here commands the motors.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

COLUMNS = ("FL", "FR", "RL", "RR")
COUNTS_PER_WHEEL_REV = 520.0        # mirror of firmware config.h, display only

# A pause longer than this separates two bursts. Comfortably above the pauses a
# hand makes mid-roll, comfortably below a deliberate "stop and count" pause.
BURST_GAP_S = 1.2
# Ignore segments smaller than this: hand tremor and quadrature dither on a
# stationary wheel produce a few counts that are not a deliberate turn.
MIN_BURST_COUNTS = 60


def segment(samples, idx):
    """Split one column's motion into bursts. Returns [(t_start, t_end, net)]."""
    changes = [
        (samples[k][0], samples[k][idx + 1] - samples[k - 1][idx + 1])
        for k in range(1, len(samples))
        if samples[k][idx + 1] != samples[k - 1][idx + 1]
    ]
    if not changes:
        return []

    bursts = []
    start = last = changes[0][0]
    net = 0
    for t, d in changes:
        if t - last > BURST_GAP_S:
            bursts.append((start, last, net))
            start, net = t, 0
        net += d
        last = t
    bursts.append((start, last, net))
    return [b for b in bursts if abs(b[2]) >= MIN_BURST_COUNTS]


class Capture(Node):
    def __init__(self, topic):
        super().__init__("encoder_id")
        self.samples = []
        self.t0 = time.monotonic()
        self.create_subscription(Int32MultiArray, topic, self._cb, 100)

    def _cb(self, msg):
        d = list(msg.data)[:4]
        if len(d) == 4:
            self.samples.append((time.monotonic() - self.t0, *d))


def report(samples):
    if len(samples) < 2:
        print("No data received. Is the agent running?  ros2 topic hz /tiny/wheel_ticks")
        return

    print(f"\n{len(samples)} samples over {samples[-1][0]:.1f} s\n")
    print(f"{'column':<8}{'bursts':>7}{'net counts':>13}{'revs':>9}   sign")
    print("-" * 52)

    found = {}
    for i, name in enumerate(COLUMNS):
        bursts = segment(samples, i)
        net = samples[-1][i + 1] - samples[0][i + 1]
        sign = "+ (counts UP)" if net > 0 else "- (counts DOWN)" if net < 0 else "(no motion)"
        print(f"{name:<8}{len(bursts):>7}{net:>13d}{net / COUNTS_PER_WHEEL_REV:>9.2f}   {sign}")
        for b in bursts:
            print(f"          {b[0]:6.1f}s -> {b[1]:6.1f}s   {b[2]:+7d}")
        if bursts:
            found[len(bursts)] = (name, net)

    print("\n" + "=" * 52)
    expected = {1: "FL", 2: "FR", 3: "RL", 4: "RR"}
    ok = True
    for n_bursts, physical in sorted(expected.items()):
        if n_bursts not in found:
            print(f"  {physical}: NO {n_bursts}-burst column found -- wheel not turned, "
                  f"or its encoder is dead")
            ok = False
            continue
        column, net = found[n_bursts]
        enc_dir = +1 if net > 0 else -1
        tag = "OK" if column == physical else f"MISMATCH -> wired to the {column} pins"
        print(f"  physical {physical}  ->  {column} column   ENC_DIR = {enc_dir:+d}   {tag}")
        if column != physical:
            ok = False

    extra = set(found) - set(expected)
    if extra:
        print(f"  unexpected burst counts {sorted(extra)} -- a wheel was turned the "
              f"wrong number of times; re-run")
        ok = False
    print("=" * 52)
    print("  All four map straight through; only ENC_DIR may need editing."
          if ok else "  Mapping needs fixing in config.h -- see above.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/tiny/wheel_ticks")
    ap.add_argument("--duration", type=float, default=180.0, help="seconds to record")
    args = ap.parse_args()

    rclpy.init()
    node = Capture(args.topic)
    print(f"Recording {args.duration:.0f} s from {args.topic}.")
    print("Roll each wheel FORWARD:  FL x1 burst, FR x2, RL x3, RR x4."
          " Pause ~2 s between bursts.")
    sys.stdout.flush()

    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    report(node.samples)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
