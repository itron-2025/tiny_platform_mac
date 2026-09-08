#!/usr/bin/env python3
"""Gamepad teleoperation for tiny_platform_mac: sensor_msgs/Joy -> geometry_msgs/Twist.

Data flow
    Pro Controller --bluetooth--> joy_node --/tiny/joy--> THIS NODE
        --/tiny/cmd_vel--> micro-ROS agent --USB serial--> ESP32

The flashed firmware subscribes to cmd_vel directly and runs the mecanum inverse
kinematics on board, so the Twist published here is the final command:
linear.x forward [m/s], linear.y left [m/s], angular.z counter-clockwise [rad/s]
(REP-103).

EVERYTHING LIVES UNDER THE /tiny NAMESPACE. That is not decoration: the Tomcat
chassis project runs on this same host, on the same DDS domain, publishing a
bare /cmd_vel from its own gamepad. Without the namespace its deadman would
drive this robot. ROS_DOMAIN_ID cannot be used for the split -- the prebuilt
micro-ROS agent ignores it (see CLAUDE.md 3.10).

Default stick assignment ("left stick moves, right stick turns"):
    left stick  up/down    -> linear.x   (forward / backward)
    left stick  left/right -> linear.y   (strafe left / right)
    right stick left/right -> angular.z  (rotate CCW / CW)

Two control modes, chosen by `enable_button`:

    enable_button >= 0   DEADMAN. A Twist is only produced while that button is
                         held; releasing it stops the robot immediately.
    enable_button == -1  DIRECT. The sticks drive the robot with nothing held.
                         Convenient, and the user asked for it -- but understand
                         what it costs: a dropped controller, a stuck stick or
                         simply walking away no longer stops the robot. The only
                         remaining stops are the /tiny/joy watchdog below, the
                         firmware's own 500 ms cmd_vel timeout, and the power
                         switch. In this mode joy_node's `deadzone` is what
                         keeps stick drift from creeping the robot, so do not
                         set it to 0.

Safety behaviour (this is a physical robot):
    * DEADMAN: see above. In DIRECT mode this protection is off by choice.
    * ARM ON CENTRE: in DIRECT mode the node refuses to drive until it has seen
      all three axes near zero at least once. Without it, starting the node (or
      recovering from a dropout) while a stick happens to be deflected would
      lurch the robot the instant it comes up. Costs nothing in normal use:
      the sticks are centred when you are not touching them.
    * WATCHDOG: if /tiny/joy goes silent for `joy_timeout` seconds while enabled
      (controller out of range, flat battery, joy_node crashed) a zero Twist is
      published and the node disarms until the button is pressed again. This
      needs joy_node's autorepeat_rate > 0, because otherwise joy_node only
      publishes on change and silence is indistinguishable from a held stick.
    * SHUTDOWN: Ctrl-C / SIGTERM publishes a burst of zero Twists before exit.
    * The command is re-published at `publish_rate` Hz while enabled, which also
      keeps feeding the firmware's own 500 ms watchdog.
    * A /tiny/joy message too short for the configured indices is treated as
      "deadman released" rather than indexed into. That case is real: SDL2 falls
      back to a different backend with a different layout if it cannot open the
      controller through HIDAPI.

All indices and scales are ROS parameters (config/teleop_params.yaml); nothing
controller-specific is hard-coded. Run `ros2 run tiny_teleop joy_probe` to find
the indices for a different controller. Negative scales invert an axis.
"""

import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy

# Ceiling the firmware will honour: MAX_WHEEL_RAD_S x wheel radius. Above this
# the ESP32 scales the whole command down, which silently turns the top of the
# stick travel into dead zone. Only used to WARN.
# KEEP IN STEP WITH firmware/tiny_open/config.h -- these two are one number
# split across two languages, and nothing checks that they agree.
_FIRMWARE_LINEAR_CAP = 45.0 * 0.040   # 1.8 m/s

# Below roughly this speed the open-loop feedforward is no longer accurate: the
# duty->speed line is fitted over the moving region and the wheels sit in their
# stiction band down here (measured 2026-09-08: up to 25% error at 0.13 m/s,
# under 10% at 0.4 m/s). Warn if the scales put the whole stick down there,
# because the symptom -- the robot yawing slightly instead of driving straight --
# reads like a calibration fault rather than a known open-loop limit.
_OPEN_LOOP_GOOD_ABOVE = 0.15          # m/s


class JoyTeleop(Node):
    """Turns /tiny/joy into /tiny/cmd_vel with a deadman switch, watchdog and turbo."""

    def __init__(self):
        super().__init__('joy_teleop')

        # --- Axis / button indices (controller layout) -----------------------
        self.declare_parameter('axis_linear_x', 1, ParameterDescriptor(
            description='Joy axis index for forward/backward.'))
        self.declare_parameter('axis_linear_y', 0, ParameterDescriptor(
            description='Joy axis index for strafe left/right.'))
        self.declare_parameter('axis_angular_z', 2, ParameterDescriptor(
            description='Joy axis index for rotation.'))
        self.declare_parameter('enable_button', 9, ParameterDescriptor(
            description='Deadman: hold this button to drive. -1 = DIRECT mode, '
                        'sticks drive with nothing held (no deadman).'))
        self.declare_parameter('center_epsilon', 0.15, ParameterDescriptor(
            description='DIRECT mode only: |axis| under this counts as centred, '
                        'for the arm-on-centre check.'))
        self.declare_parameter('turbo_button', 10, ParameterDescriptor(
            description='Hold with the deadman for the turbo scales. -1 disables.'))

        # --- Scales: joy axis (-1..1) times scale = command -------------------
        self.declare_parameter('scale_linear_x', 0.25)        # [m/s]
        self.declare_parameter('scale_linear_y', 0.25)        # [m/s]
        self.declare_parameter('scale_angular_z', 1.0)        # [rad/s]
        self.declare_parameter('scale_linear_x_turbo', 0.5)   # [m/s]
        self.declare_parameter('scale_linear_y_turbo', 0.5)   # [m/s]
        self.declare_parameter('scale_angular_z_turbo', 2.0)  # [rad/s]

        # --- Timing / safety ---------------------------------------------------
        self.declare_parameter('publish_rate', 20.0)          # [Hz]
        self.declare_parameter('joy_timeout', 1.0)            # [s]
        self.declare_parameter('stop_burst', 3)
        self.declare_parameter('idle_quiet_after', 1.0, ParameterDescriptor(
            description='Seconds of a centred stick after which this node stops '
                        'publishing at all, so it does not fight other '
                        'publishers on cmd_vel. 0 = always publish.'))

        # Cached copies so the 20 Hz timer does not go through the parameter API.
        self._cfg = {}
        for name in ('axis_linear_x', 'axis_linear_y', 'axis_angular_z',
                     'enable_button', 'turbo_button',
                     'scale_linear_x', 'scale_linear_y', 'scale_angular_z',
                     'scale_linear_x_turbo', 'scale_linear_y_turbo',
                     'scale_angular_z_turbo', 'joy_timeout', 'stop_burst',
                     'center_epsilon', 'idle_quiet_after'):
            self._cfg[name] = self.get_parameter(name).value
        self._warn_scales()
        self.add_on_set_parameters_callback(self._on_param_change)

        # --- State -------------------------------------------------------------
        self._lock = threading.Lock()
        self._enabled = False
        self._turbo = False
        self._target = (0.0, 0.0, 0.0)
        self._last_joy_time = None
        self._layout_warned = False
        # DIRECT mode only: becomes True once the sticks have been seen centred.
        self._centered = False
        # When the command last became non-zero, for the idle-quiet logic below.
        self._last_nonzero = 0.0

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._joy_sub = self.create_subscription(Joy, 'joy', self._on_joy, 10)

        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0.0:
            self.get_logger().warn('publish_rate must be > 0; using 20 Hz')
            rate = 20.0
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        if self._cfg['enable_button'] < 0:
            how = ('DIRECT mode: no deadman, the sticks drive. Centre them once '
                   'to arm. Turbo: button %d' % self._cfg['turbo_button'])
        else:
            how = ('hold button %d to drive (turbo: button %d)'
                   % (self._cfg['enable_button'], self._cfg['turbo_button']))
        self.get_logger().info(
            'joy_teleop ready: %s. axes x/y/yaw = %d/%d/%d, '
            'scales %.2f/%.2f m/s, %.2f rad/s'
            % (how, self._cfg['axis_linear_x'], self._cfg['axis_linear_y'],
               self._cfg['axis_angular_z'], self._cfg['scale_linear_x'],
               self._cfg['scale_linear_y'], self._cfg['scale_angular_z']))

    # ------------------------------------------------------------------ params
    def _on_param_change(self, params):
        for p in params:
            if p.name not in self._cfg:
                continue
            if p.name.startswith(('axis_', 'enable_', 'turbo_', 'stop_')) and \
                    not isinstance(p.value, int):
                return SetParametersResult(
                    successful=False, reason='%s must be an integer' % p.name)
            if p.name.startswith(('scale_', 'joy_')) and \
                    not isinstance(p.value, (int, float)):
                return SetParametersResult(
                    successful=False, reason='%s must be a number' % p.name)
        with self._lock:
            for p in params:
                if p.name in self._cfg:
                    self._cfg[p.name] = p.value
        self._warn_scales()
        return SetParametersResult(successful=True)

    def _warn_scales(self):
        for name in ('scale_linear_x', 'scale_linear_y',
                     'scale_linear_x_turbo', 'scale_linear_y_turbo'):
            v = abs(self._cfg[name])
            if v > _FIRMWARE_LINEAR_CAP + 1e-6:
                self.get_logger().warn(
                    '%s = %.2f m/s exceeds the firmware ceiling of %.2f m/s; the '
                    'ESP32 scales it down and the top of the stick goes dead.'
                    % (name, self._cfg[name], _FIRMWARE_LINEAR_CAP))
            elif 0.0 < v < _OPEN_LOOP_GOOD_ABOVE:
                self.get_logger().warn(
                    '%s = %.2f m/s is inside the open-loop stiction band (below '
                    '%.2f m/s the wheels track to about +/-25%%); expect a slight '
                    'yaw on start. Not a fault.'
                    % (name, self._cfg[name], _OPEN_LOOP_GOOD_ABOVE))

    # -------------------------------------------------------------------- joy
    def _on_joy(self, msg: Joy):
        cfg = self._cfg
        need_axes = max(cfg['axis_linear_x'], cfg['axis_linear_y'],
                        cfg['axis_angular_z']) + 1
        # -1 means "not used", so it must not drag the requirement to 0.
        need_buttons = max(cfg['enable_button'], cfg['turbo_button'], -1) + 1
        if len(msg.axes) < need_axes or len(msg.buttons) < need_buttons:
            # Wrong controller layout (e.g. SDL fell back to the evdev backend).
            # Never drive on a guess: treat as "deadman released".
            if not self._layout_warned:
                self.get_logger().error(
                    '/tiny/joy has %d axes and %d buttons but the parameters need '
                    'at least %d and %d -- check config/teleop_params.yaml (run '
                    '`ros2 run tiny_teleop joy_probe`). Refusing to drive.'
                    % (len(msg.axes), len(msg.buttons), need_axes, need_buttons))
                self._layout_warned = True
            self._disarm(publish_zero=True)
            return

        if cfg['enable_button'] >= 0:
            enabled = msg.buttons[cfg['enable_button']] == 1
        else:
            # DIRECT mode. Arm only after the sticks have been seen centred, so
            # a node restart with a deflected stick cannot lurch the robot.
            eps = float(cfg['center_epsilon'])
            centred = all(
                abs(msg.axes[cfg[a]]) <= eps
                for a in ('axis_linear_x', 'axis_linear_y', 'axis_angular_z'))
            if centred and not self._centered:
                self._centered = True
                self._last_nonzero = time.monotonic()   # emit a short stop tail
                self.get_logger().info('sticks centred -> armed (DIRECT mode)')
            enabled = self._centered

        turbo = cfg['turbo_button'] >= 0 and msg.buttons[cfg['turbo_button']] == 1

        if enabled:
            if turbo:
                sx, sy, sz = (cfg['scale_linear_x_turbo'],
                              cfg['scale_linear_y_turbo'],
                              cfg['scale_angular_z_turbo'])
            else:
                sx, sy, sz = (cfg['scale_linear_x'], cfg['scale_linear_y'],
                              cfg['scale_angular_z'])
            target = (msg.axes[cfg['axis_linear_x']] * sx,
                      msg.axes[cfg['axis_linear_y']] * sy,
                      msg.axes[cfg['axis_angular_z']] * sz)
        else:
            target = (0.0, 0.0, 0.0)

        with self._lock:
            was_enabled = self._enabled
            self._enabled = enabled
            self._turbo = turbo
            self._target = target
            self._last_joy_time = time.monotonic()

        if was_enabled and not enabled:
            # Stop NOW, do not wait for the timer tick.
            self._publish(0.0, 0.0, 0.0)
            if cfg['enable_button'] >= 0:
                self.get_logger().info('deadman released -> stop')
        elif enabled and not was_enabled and cfg['enable_button'] >= 0:
            self.get_logger().info(
                'deadman pressed -> driving%s' % (' (TURBO)' if turbo else ''))

    # ------------------------------------------------------------------ timer
    def _on_timer(self):
        with self._lock:
            enabled = self._enabled
            target = self._target
            last = self._last_joy_time
            timeout = float(self._cfg['joy_timeout'])
        if not enabled:
            return
        if last is not None and timeout > 0.0 and (time.monotonic() - last) > timeout:
            self.get_logger().warn(
                'no /tiny/joy for %.1f s while driving -> stop (controller '
                'disconnected? joy_node needs autorepeat_rate > 0)' % timeout)
            self._disarm(publish_zero=True)
            return

        # GO QUIET WHEN IDLE. In DIRECT mode this node is armed permanently, so
        # without this it publishes a zero Twist 20 times a second forever -- and
        # then every calibration tool that drives cmd_vel is fighting it, at
        # equal rate, for the same topic.
        #
        # That is not a cosmetic conflict. The firmware treats a zero command as
        # "stop", which RESETS the PID integrator; interleaving zeros with a real
        # command clears the integrator every other tick and the closed loop
        # never accumulates. Measured 2026-09-08: a yaw command tracked to 54%
        # error with this node idling in the background, and to 3% with it
        # stopped. The controller looked broken; it was being interrupted.
        #
        # A short tail of zeros still goes out after the sticks return to centre,
        # so a release is always delivered promptly. After that the firmware's
        # own 500 ms watchdog is what holds the robot stopped -- which is what it
        # is for.
        now = time.monotonic()
        if any(abs(v) > 1e-6 for v in target):
            self._last_nonzero = now
        else:
            quiet_after = float(self._cfg['idle_quiet_after'])
            if quiet_after > 0.0 and (now - self._last_nonzero) > quiet_after:
                return

        self._publish(*target)

    # ---------------------------------------------------------------- helpers
    def _disarm(self, publish_zero: bool):
        with self._lock:
            self._enabled = False
            self._turbo = False
            self._target = (0.0, 0.0, 0.0)
            # Re-require a centred stick before driving again. After a dropout
            # the last stick position is unknown, and resuming straight into it
            # is exactly the lurch the arm-on-centre check exists to prevent.
            self._centered = False
        if publish_zero:
            self._publish(0.0, 0.0, 0.0)

    def _publish(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def send_stop_burst(self):
        n = max(1, int(self._cfg['stop_burst']))
        for i in range(n):
            self._publish(0.0, 0.0, 0.0)
            if i + 1 < n:
                time.sleep(0.05)
        self.get_logger().info('sent %d zero cmd_vel on shutdown' % n)


def main(args=None):
    # rclpy's own SIGINT handler tears the context down BEFORE we could publish
    # the final zero command, so install our own instead.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = JoyTeleop()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        try:
            node.send_stop_burst()
        except Exception as exc:  # noqa: BLE001 - never skip the cleanup below
            node.get_logger().error('could not publish stop on shutdown: %s' % exc)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
