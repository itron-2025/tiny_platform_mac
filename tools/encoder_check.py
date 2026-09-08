#!/usr/bin/env python3
"""Live encoder read-out for hand-turning the wheels. No motor power needed.

Run it, then turn each wheel BY HAND and watch which column moves. In one pass
this answers four separate questions that would otherwise need four experiments:

  1. Is every encoder alive?          a column that never moves is not wired /
                                      not powered / missing its pull-up
  2. Which encoder is which wheel?    turn the front-left wheel, see which
                                      column reacts
  3. Which way does each one count?   forward-rolling the wheel should count UP;
                                      if it counts down, ENC_DIR[i] = -1
  4. COUNTS_PER_WHEEL_REV             turn exactly one revolution and read the
                                      "since reset" figure -- it should be 520

Usage (inside the container, agent already running):
    python3 tools/encoder_check.py            # live table
    python3 tools/encoder_check.py --reset    # zero the "since reset" counters

Press Ctrl-C to stop. Nothing here commands the motors.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

WHEELS = ("FL", "FR", "RL", "RR")

# Must match COUNTS_PER_WHEEL_REV in firmware/*/config.h. Only used to show the
# revolution estimate; a mismatch here changes nothing on the robot.
COUNTS_PER_WHEEL_REV = 520.0


class EncoderCheck(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("encoder_check")
        self._base = None          # counts at the last reset
        self._last = None          # previous sample, for the "moving?" marker
        self._painted = 0.0        # last redraw time
        # On a terminal, repaint one line in place. When the output is piped or
        # redirected, \r does nothing useful and 50 Hz would produce a wall of
        # text, so only emit a real line when a count actually changes.
        self._tty = sys.stdout.isatty()
        self.create_subscription(Int32MultiArray, topic, self._on_ticks, 10)
        self.get_logger().info(f"listening on {topic} -- turn a wheel by hand")

    def _on_ticks(self, msg: Int32MultiArray) -> None:
        data = list(msg.data)[:4]
        if len(data) < 4:
            return
        if self._base is None:
            self._base = list(data)
            self._last = list(data)

        cells = []
        for i, name in enumerate(WHEELS):
            delta = _wrap_int32(data[i] - self._base[i])
            moving = data[i] != self._last[i]
            revs = delta / COUNTS_PER_WHEEL_REV
            mark = "*" if moving else " "
            cells.append(f"{name}{mark}{delta:+8d} ({revs:+6.2f} rev)")

        changed = data != self._last
        self._last = list(data)
        line = " | ".join(cells)

        if self._tty:
            # Cap the repaint rate: at 50 Hz the numbers are unreadable anyway.
            now = time.monotonic()
            if now - self._painted < 0.1:
                return
            self._painted = now
            # \r keeps it on one line; trailing spaces clear a longer previous one.
            sys.stdout.write("\r" + line + "   ")
        elif changed:
            sys.stdout.write(line + "\n")
        else:
            return
        sys.stdout.flush()

    def reset(self) -> None:
        self._base = None


def _wrap_int32(v: int) -> int:
    """Fold a difference back into int32 range so a counter wrap is not read as
    four billion counts. The firmware publishes a cumulative int32."""
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v >= (1 << 31) else v


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/tiny/wheel_ticks")
    ap.add_argument("--reset", action="store_true",
                    help="zero the counters once at startup (they zero on the "
                         "first sample anyway; use this to re-zero mid-session)")
    args = ap.parse_args()

    rclpy.init()
    node = EncoderCheck(args.topic)
    if args.reset:
        node.reset()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what a SIGTERM looks like from inside
        # rclpy (e.g. `timeout 5 python3 ...`). Both are normal exits here.
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
