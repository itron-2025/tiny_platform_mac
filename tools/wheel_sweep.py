#!/usr/bin/env python3
"""Drive one wheel at a time and report what each encoder did. Bring-up stage A/B.

Publishes raw duty on /tiny/wheel_duty (which bypasses the inverse kinematics
and the deadband boost, so what is asked for is what the motor gets) and records
/tiny/wheel_ticks throughout. For each wheel it prints the counts every column
moved during that wheel's window.

WHAT THIS CAN AND CANNOT SETTLE, so the output is not over-read:

  CAN: that each of the four drive channels actually works; which encoder column
       responds to which drive channel; and the SIGN relation between a positive
       duty and that encoder's count direction (i.e. whether MOTOR_DIR and
       ENC_DIR agree with each other).

  CANNOT: which PHYSICAL wheel a channel drives, or whether "positive duty"
       means the robot moves forward. On this board each motor's power and
       encoder share one connector, so a connector plugged into the wrong port
       moves BOTH together and stays perfectly self-consistent -- invisible from
       the data. Those two facts need human eyes on the robot, or
       tools/encoder_id.py.

SAFETY: DUTY defaults deliberately low and each burst is short. The firmware
also clamps everything to DUTY_LIMIT (config.h) and stops the wheels if commands
stop arriving for CMD_TIMEOUT_MS. Elevate the chassis before raising --duty.

Usage (inside the container, agent running):
    python3 tools/wheel_sweep.py                 # all four, gentle
    python3 tools/wheel_sweep.py --wheel FL      # just one
    python3 tools/wheel_sweep.py --duty 250 --on 3.0
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

WHEELS = ("FL", "FR", "RL", "RR")
PUBLISH_HZ = 20.0        # firmware watchdog is 500 ms; 20 Hz is comfortably inside


class Sweep(Node):
    def __init__(self, duty_topic, tick_topic):
        super().__init__("wheel_sweep")
        self.pub = self.create_publisher(Float32MultiArray, duty_topic, 10)
        self.ticks = None
        self.create_subscription(Int32MultiArray, tick_topic, self._cb, 50)

    def _cb(self, msg):
        d = list(msg.data)[:4]
        if len(d) == 4:
            self.ticks = d

    def _spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _send(self, duties):
        m = Float32MultiArray()
        m.data = [float(x) for x in duties]
        self.pub.publish(m)

    def hold(self, duties, seconds):
        """Publish `duties` at PUBLISH_HZ for `seconds`, spinning in between."""
        period = 1.0 / PUBLISH_HZ
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self._send(duties)
            self._spin(period)

    def wait_for_ticks(self, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self.ticks is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ticks is not None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duty", type=float, default=180.0,
                    help="signed duty, 0..1023 (firmware clamps to DUTY_LIMIT)")
    ap.add_argument("--on", type=float, default=2.0, help="seconds driving")
    ap.add_argument("--off", type=float, default=3.0, help="seconds stopped between wheels")
    ap.add_argument("--wheel", choices=WHEELS, help="only this wheel")
    ap.add_argument("--duty-topic", default="/tiny/wheel_duty")
    ap.add_argument("--tick-topic", default="/tiny/wheel_ticks")
    ap.add_argument("--start-delay", type=float, default=0.0,
                    help="seconds to wait before the first wheel, so whoever is "
                         "watching the robot has time to get into position")
    args = ap.parse_args()

    rclpy.init()
    node = Sweep(args.duty_topic, args.tick_topic)

    if not node.wait_for_ticks():
        print("No /tiny/wheel_ticks. Is the agent running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    targets = [args.wheel] if args.wheel else list(WHEELS)
    print(f"duty {args.duty:+.0f} for {args.on:.1f}s per wheel, {args.off:.1f}s between.")
    print("WATCH THE ROBOT: note which wheel moves, and which way it rolls.\n")
    sys.stdout.flush()

    if args.start_delay > 0:
        # Publish zeros through the countdown: it keeps the firmware watchdog fed
        # so the first wheel starts crisply instead of after a watchdog recovery.
        print(f"starting in {args.start_delay:.0f}s ...", flush=True)
        node.hold([0, 0, 0, 0], args.start_delay)

    results = []
    try:
        # Settle: hold zero so the firmware sees a fresh command and any previous
        # publisher's watchdog has expired.
        node.hold([0, 0, 0, 0], 1.0)

        for name in targets:
            i = WHEELS.index(name)
            before = list(node.ticks)
            print(f"--> driving {name} (channel {i}) ...", flush=True)

            duties = [0.0] * 4
            duties[i] = args.duty
            node.hold(duties, args.on)

            # Stop, then let the wheel coast to rest before reading.
            node.hold([0, 0, 0, 0], args.off)
            after = list(node.ticks)

            delta = [after[k] - before[k] for k in range(4)]
            results.append((name, delta))
            cells = "  ".join(f"{WHEELS[k]}={delta[k]:+6d}" for k in range(4))
            print(f"    {cells}", flush=True)
    finally:
        # Always leave the wheels commanded to zero, whatever happened above.
        for _ in range(5):
            node._send([0, 0, 0, 0])
            time.sleep(0.02)

    print("\n" + "=" * 60)
    for name, delta in results:
        i = WHEELS.index(name)
        own = delta[i]
        others = {WHEELS[k]: delta[k] for k in range(4) if k != i and abs(delta[k]) > 30}
        if own == 0 and not others:
            print(f"  {name}: NO MOTION AT ALL -- dead channel, or duty below the "
                  f"motor's deadband (try --duty 300)")
        elif own == 0 and others:
            print(f"  {name}: drove, but the counts landed on {others} -- motor and "
                  f"encoder of this channel are NOT the same wheel")
        else:
            sign = "+" if own > 0 else "-"
            note = f"  (also moved: {others})" if others else ""
            print(f"  {name}: {own:+d} counts on its own encoder, sign {sign}{note}")
    print("=" * 60)
    print("  A positive duty should make the robot go FORWARD and the count go UP.")
    print("  Wrong way round on the wheel  -> MOTOR_DIR[i] = -1")
    print("  Wheel right, count going down -> ENC_DIR[i]   = -1")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
