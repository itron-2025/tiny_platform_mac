#!/usr/bin/env python3
"""Measure COUNTS_PER_WHEEL_REV by hand-turning each wheel a whole number of turns.

WHY THIS MEASUREMENT IS NOT OPTIONAL, and why no amount of software can replace
it: COUNTS_PER_WHEEL_REV scales the PID's measurement AND its setpoint by the
same factor, and scales the odometry by that factor again in the other
direction. Get it wrong and the control loop still tracks its target perfectly
while every wheel turns at the wrong speed, and the odometry under-reports by
exactly the compensating amount. Every self-consistency check passes. Only a
physical measurement -- a mark on the wheel, or a tape measure on the floor --
can see it. The Tomcat chassis ran a whole benchmarking campaign on a value that
was 2.5x wrong.

METHOD. Put a mark on each wheel. With the agent running and no motor command
(the firmware coasts the motors, so the wheels turn freely), roll each wheel by
hand through a whole number of complete turns, in the direction that drives the
robot FORWARD, pausing between wheels. Then:

    counts_per_rev = |counts in that burst| / turns

Turn 3 times rather than once: a +/-10 degree error in lining the mark back up
is +/-2.8% over one turn but only +/-0.9% over three, which is the difference
between resolving 520 from 530 and not.

Usage (inside the container, agent running):
    python3 tools/counts_per_rev.py --turns 3 --duration 180
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

COLUMNS = ("FL", "FR", "RL", "RR")
NOMINAL = 520.0                 # 13 PPR x 4 (quadrature) x 10:1 gearbox
BURST_GAP_S = 1.2               # a pause longer than this separates two bursts
MIN_BURST_COUNTS = 60           # ignore hand tremor / quadrature dither


def segment(samples, idx):
    changes = [
        (samples[k][0], samples[k][idx + 1] - samples[k - 1][idx + 1])
        for k in range(1, len(samples))
        if samples[k][idx + 1] != samples[k - 1][idx + 1]
    ]
    if not changes:
        return []
    bursts, start, last, net = [], changes[0][0], changes[0][0], 0
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
        super().__init__("counts_per_rev")
        self.samples = []
        self.t0 = time.monotonic()
        self.create_subscription(Int32MultiArray, topic, self._cb, 100)

    def _cb(self, msg):
        d = list(msg.data)[:4]
        if len(d) == 4:
            self.samples.append((time.monotonic() - self.t0, *d))


def plausible_turn_count(counts, turns):
    """If the implied counts-per-rev is close to NOMINAL * k for a small integer
    or simple fraction k, the wheel was probably turned k*turns times, not turns.
    Saying so is far more useful than printing a wrong constant."""
    implied = abs(counts) / turns
    ratio = implied / NOMINAL
    for k in (0.25, 1 / 3, 0.5, 2 / 3, 1, 1.5, 2, 3, 4, 5):
        if abs(ratio - k) < 0.06 and abs(k - 1) > 1e-9:
            return f"looks like {k * turns:g} turns, not {turns:g}"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/tiny/wheel_ticks")
    ap.add_argument("--turns", type=float, default=3.0,
                    help="whole turns given to EACH wheel (default 3)")
    ap.add_argument("--duration", type=float, default=180.0)
    args = ap.parse_args()

    rclpy.init()
    node = Capture(args.topic)
    print(f"Recording {args.duration:.0f}s. Turn each wheel {args.turns:g} full "
          f"turns FORWARD, pausing between wheels.")
    sys.stdout.flush()

    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    s = node.samples
    if len(s) < 2:
        print("No data. Is the agent running?")
    else:
        print(f"\n{len(s)} samples over {s[-1][0]:.1f}s   (nominal {NOMINAL:g} counts/rev)\n")
        print(f"{'wheel':<7}{'bursts':>7}{'counts':>10}{'counts/rev':>12}{'vs 520':>9}   note")
        print("-" * 68)
        good = []
        for i, name in enumerate(COLUMNS):
            bursts = segment(s, i)
            if not bursts:
                print(f"{name:<7}{0:>7}{'-':>10}{'-':>12}{'-':>9}   NOT TURNED")
                continue
            total = sum(b[2] for b in bursts)
            implied = abs(total) / (args.turns * len(bursts))
            err = (implied / NOMINAL - 1.0) * 100.0
            note = plausible_turn_count(total / len(bursts), args.turns) or ""
            if total < 0:
                note = (note + "  " if note else "") + "counted DOWN (rolled backward?)"
            print(f"{name:<7}{len(bursts):>7}{total:>10d}{implied:>12.1f}{err:>+8.1f}%   {note}")
            if not note:
                good.append(implied)

        print("-" * 68)
        if good:
            mean = sum(good) / len(good)
            spread = (max(good) - min(good)) / mean * 100 if len(good) > 1 else 0.0
            print(f"  mean of {len(good)} clean wheels: {mean:.1f} counts/rev "
                  f"(spread {spread:.1f}%)")
            print(f"  -> put this in COUNTS_PER_WHEEL_REV (firmware config.h)")
            if abs(mean / NOMINAL - 1.0) < 0.03:
                print(f"  the 13 PPR x 4 x 10:1 = {NOMINAL:g} figure is confirmed.")
        else:
            print("  nothing clean to average -- check the notes above and re-run.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
