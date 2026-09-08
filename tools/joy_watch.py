#!/usr/bin/env python3
"""Live view of the whole teleop chain, for answering "is my button press even
getting in?".

Shows, updating in place:
    every axis and every button of /tiny/joy, with its INDEX
    whether the deadman index the teleop is configured for is currently held
    the /tiny/cmd_vel that came out
    the wheel speeds that resulted

and, below that, a scrolling log line for every button press and release, so the
history survives even if you were looking at the robot instead of the screen.

The point of showing indices rather than names: the mapping from a physical
button to an index is NOT a fixed property of the controller. The same Pro
Controller enumerates differently through SDL2's HIDAPI and evdev backends, so
the only trustworthy answer is the one this prints for the controller you are
actually holding. If the button you press is not the index in
config/teleop_params.yaml, that is the bug -- fix the YAML, not the wiring.

Usage (inside the container):
    python3 tools/joy_watch.py
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

WHEELS = ("FL", "FR", "RL", "RR")
COUNTS_PER_WHEEL_REV = 574.0

# Pro Controller through SDL2's HIDAPI backend. Shown only as a hint next to the
# index -- the index is what matters and what the parameters use.
BUTTON_HINTS = {
    0: "A", 1: "B", 2: "X", 3: "Y", 4: "-", 5: "Home", 6: "+",
    7: "Lstick", 8: "Rstick", 9: "L", 10: "R",
    11: "Dup", 12: "Ddown", 13: "Dleft", 14: "Dright", 15: "Capture",
}
AXIS_HINTS = {
    0: "Lx (+left)", 1: "Ly (+up)", 2: "Rx (+left)",
    3: "Ry (+up)", 4: "ZL", 5: "ZR",
}
BLOCK_LINES = 9


class JoyWatch(Node):
    def __init__(self, deadman, turbo):
        super().__init__("joy_watch")
        self.deadman = deadman
        self.turbo = turbo
        self.joy = None
        self.n_joy = 0
        self.cmd = (0.0, 0.0, 0.0)
        self.n_cmd = 0
        self.omega = [0.0] * 4
        self._prev_buttons = None
        self._prev_ticks = None
        self._prev_t = 0.0
        self._painted = False
        self._last_paint = 0.0
        self._log = []

        self.create_subscription(Joy, "/tiny/joy", self._on_joy, 10)
        self.create_subscription(Twist, "/tiny/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Int32MultiArray, "/tiny/wheel_ticks", self._on_ticks, 50)

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.n_cmd += 1

    def _on_ticks(self, m):
        d = list(m.data)[:4]
        if len(d) != 4:
            return
        now = time.monotonic()
        if self._prev_ticks is not None and now - self._prev_t > 0.2:
            dt = now - self._prev_t
            self.omega = [((d[i] - self._prev_ticks[i]) / COUNTS_PER_WHEEL_REV)
                          * 2 * math.pi / dt for i in range(4)]
            self._prev_ticks, self._prev_t = d, now
        elif self._prev_ticks is None:
            self._prev_ticks, self._prev_t = d, now

    def _on_joy(self, m):
        self.joy = m
        self.n_joy += 1
        b = list(m.buttons)
        if self._prev_buttons is not None:
            for i, (o, n) in enumerate(zip(self._prev_buttons, b)):
                if n != o:
                    hint = BUTTON_HINTS.get(i, "?")
                    self._log.append(
                        f"  buttons[{i}] ({hint}) {'PRESSED' if n else 'released'}")
        self._prev_buttons = b

    # ------------------------------------------------------------------ paint
    def paint(self):
        now = time.monotonic()
        if now - self._last_paint < 0.1:
            return
        self._last_paint = now

        out = []
        if self.joy is None:
            out.append("waiting for /tiny/joy ... (is joy_node running?)")
            out += [""] * (BLOCK_LINES - 1)
        else:
            j = self.joy
            axes = "  ".join(
                f"[{i}]{v:+5.2f}" for i, v in enumerate(j.axes))
            out.append(f"axes    {axes}")
            out.append("        " + "  ".join(
                f"{AXIS_HINTS.get(i, ''):>9.9}" for i in range(len(j.axes))))

            pressed = [i for i, v in enumerate(j.buttons) if v]
            if pressed:
                names = "  ".join(f"[{i}]{BUTTON_HINTS.get(i, '?')}" for i in pressed)
            else:
                names = "(none)"
            out.append(f"pressed {names}")

            held = (self.deadman < len(j.buttons) and j.buttons[self.deadman] == 1)
            turbo = (0 <= self.turbo < len(j.buttons) and j.buttons[self.turbo] == 1)
            state = "HELD -> DRIVING" if held else "not held -> stopped"
            out.append(f"deadman [{self.deadman}] {state}"
                       + ("   TURBO" if held and turbo else ""))

            out.append(f"joy msgs {self.n_joy}   "
                       f"{len(j.axes)} axes / {len(j.buttons)} buttons")

        vx, vy, wz = self.cmd
        out.append(f"cmd_vel  vx{vx:+6.2f}  vy{vy:+6.2f}  wz{wz:+6.2f}"
                   f"   ({self.n_cmd} msgs)")
        out.append("wheels   " + "  ".join(
            f"{WHEELS[i]}{self.omega[i]:+6.1f}" for i in range(4)) + "  rad/s")
        out.append("-" * 70)
        out.append("hold L to drive.  Ctrl-C to quit.")

        out = (out + [""] * BLOCK_LINES)[:BLOCK_LINES]
        if self._painted:
            sys.stdout.write(f"\033[{BLOCK_LINES}A")
        for line in out:
            sys.stdout.write("\033[2K" + line + "\n")
        self._painted = True

        # Flush the button event log BELOW the block, so it scrolls normally.
        if self._log:
            for line in self._log:
                sys.stdout.write("\033[2K" + line + "\n")
            self._log.clear()
            self._painted = False   # block moves down; repaint fresh next time
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deadman", type=int, default=9)
    ap.add_argument("--turbo", type=int, default=10)
    args = ap.parse_args()

    rclpy.init()
    node = JoyWatch(args.deadman, args.turbo)
    print("joy_watch: press buttons and move the sticks.\n")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.paint()
    except (KeyboardInterrupt, ExternalShutdownException):
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
