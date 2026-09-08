#!/usr/bin/env python3
"""Print which /tiny/joy axis or button just moved. Use it instead of trusting a
table, because the numbering is not a property of the controller alone.

The same Pro Controller enumerates differently depending on which backend SDL2
picks: through HIDAPI it presents 6 axes and 16 buttons, through evdev it
presents a different count and moves L/R and the D-pad. Which one you get
depends on whether the process can open /dev/hidraw*, which depends on how the
container was started. So: move one stick or press one button at a time and read
the answer off the robot you actually have.

Usage:
    ros2 run tiny_teleop joy_probe
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Ignore analogue drift: a stick at rest still jitters by a few 1e-3.
AXIS_THRESHOLD = 0.25


class JoyProbe(Node):
    def __init__(self):
        super().__init__('joy_probe')
        self._axes = None
        self._buttons = None
        self._announced = False
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.get_logger().info('move ONE stick or press ONE button at a time')

    def _on_joy(self, msg: Joy):
        if not self._announced:
            print(f"\n/tiny/joy reports {len(msg.axes)} axes and "
                  f"{len(msg.buttons)} buttons.")
            if len(msg.axes) == 6 and len(msg.buttons) == 16:
                print("  -> SDL2 HIDAPI backend (the usual one in this container).")
            else:
                print("  -> NOT the 6-axis/16-button HIDAPI layout. The indices in "
                      "config/teleop_params.yaml were measured on that one and "
                      "will be wrong here. Re-measure them below.")
            print()
            self._announced = True

        if self._axes is None:
            self._axes = list(msg.axes)
            self._buttons = list(msg.buttons)
            return

        for i, (old, new) in enumerate(zip(self._axes, msg.axes)):
            if abs(new - old) > AXIS_THRESHOLD:
                direction = 'positive' if new > 0 else 'negative'
                print(f"  axes[{i}] = {new:+.2f}   ({direction})")
                sys.stdout.flush()

        for i, (old, new) in enumerate(zip(self._buttons, msg.buttons)):
            if new and not old:
                print(f"  buttons[{i}] pressed")
                sys.stdout.flush()

        self._axes = list(msg.axes)
        self._buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
