#!/usr/bin/env python3
"""Robot face for tiny_platform_mac: /tiny/joy + /tiny/cmd_vel -> a web page.

    Pro Controller --> joy_node --/tiny/joy----> THIS NODE --http/SSE--> browser
                       ESP32 cmds --/tiny/cmd_vel--^

What it does
    * The four face buttons (A/B/X/Y) pick a mood while held; released = default.
    * /tiny/cmd_vel aims the eyes: the robot looks where it is going.
    * Everything else -- blinking, idle wander, the drawing itself -- lives in
      web/face.html. This node only ships {mood, look, moving} and lets the
      browser animate. CSS transitions interpolate for free; doing it here would
      mean streaming frames instead of state.

Why a raw http.server and not Flask / rosbridge / a websocket library
    Because the image has neither, and adding one means a 40-minute rebuild for
    a page that sends one small JSON object at 20 Hz. Server-Sent Events are
    plain HTTP text and EventSource reconnects on its own, so the whole
    transport is ~30 lines of stdlib.

The eye design copies the mood set of FluxGarage/RoboEyes (MIT) -- default,
happy, tired, angry, plus autoblinker and idle wander. Not its code: that is
Arduino C++ drawing into an Adafruit GFX framebuffer, and emulating GFX in a
browser would be far more work than two rounded rectangles and a few eyelids.

    ros2 run tiny_teleop face_node --ros-args -r __ns:=/tiny
    then open http://localhost:8088
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Moods, in the order they win when several buttons are held at once.
_MOODS = (('happy', 'button_happy'), ('angry', 'button_angry'),
          ('tired', 'button_tired'), ('surprised', 'button_surprised'))


def mood_from_buttons(buttons, cfg):
    """Pick the mood for a Joy button array. Nothing held -> 'default'.

    A short array is not an error here: SDL's evdev fallback reports a different
    layout, and a face that quietly stays neutral beats one that indexes off the
    end. (joy_teleop refuses to DRIVE on that same condition -- this node is not
    safety-critical, so it just shrugs.)
    """
    for mood, key in _MOODS:
        i = cfg[key]
        if 0 <= i < len(buttons) and buttons[i]:
            return mood
    return 'default'


def look_from_twist(vx, vy, wz, lin_scale, ang_scale):
    """Twist -> where the eyes point, as (x, y) each in [-1, 1].

    REP-103 says +y is LEFT and +z is counter-clockwise, while a screen's +x is
    RIGHT -- hence the negation. Yaw counts as sideways looking too: spinning
    left should throw the eyes left, which is what makes it read as "looking
    where it is going" rather than "eyes wired to two axes".

    Scales are the normal (non-turbo) teleop scales, so a full stick is a full
    glance; turbo just saturates earlier, which is the right feel anyway.
    """
    def n(v, s):
        return max(-1.0, min(1.0, v / s)) if s > 0 else 0.0
    x = -(n(vy, lin_scale) + n(wz, ang_scale))
    return (max(-1.0, min(1.0, x)), n(vx, lin_scale))


class FaceNode(Node):
    def __init__(self):
        super().__init__('face_node')

        self.declare_parameter('port', 8088, ParameterDescriptor(
            description='HTTP port for the face page. The container is --net=host, '
                        'so this is a port on the host itself -- keep it clear of '
                        'the Tomcat project, which also runs with --net=host.'))
        self.declare_parameter('host', '0.0.0.0', ParameterDescriptor(
            description='Bind address. 0.0.0.0 so a tablet on the same network '
                        'can show the face too; 127.0.0.1 to keep it local.'))
        # Nintendo Switch Pro Controller, SDL2 HIDAPI backend: A B X Y = 0 1 2 3.
        # See the layout table in config/teleop_params.yaml. -1 disables a mood.
        self.declare_parameter('button_happy', 0)      # A
        self.declare_parameter('button_angry', 1)      # B
        self.declare_parameter('button_tired', 2)      # X
        self.declare_parameter('button_surprised', 3)  # Y
        self.declare_parameter('look_scale_linear', 0.60)   # [m/s]  full glance
        self.declare_parameter('look_scale_angular', 2.0)   # [rad/s]
        self.declare_parameter('motion_timeout', 0.5, ParameterDescriptor(
            description='Seconds without a cmd_vel after which the eyes recentre. '
                        'joy_teleop stops publishing when the sticks are centred '
                        '(CLAUDE.md 3.15), so silence IS the stopped state -- '
                        'without this the eyes would stay stuck looking sideways.'))

        self._cfg = {k: self.get_parameter(k).value for k in (
            'button_happy', 'button_angry', 'button_tired', 'button_surprised',
            'look_scale_linear', 'look_scale_angular', 'motion_timeout')}

        self._lock = threading.Lock()
        self._mood = 'default'
        self._look = (0.0, 0.0)
        self._last_cmd = 0.0

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)

        self._page = os.path.join(
            get_package_share_directory('tiny_teleop'), 'web', 'face.html')
        self._serve(self.get_parameter('host').value,
                    int(self.get_parameter('port').value))

    # ------------------------------------------------------------------- ros
    def _on_joy(self, msg: Joy):
        mood = mood_from_buttons(msg.buttons, self._cfg)
        with self._lock:
            self._mood = mood

    def _on_cmd(self, msg: Twist):
        look = look_from_twist(msg.linear.x, msg.linear.y, msg.angular.z,
                               self._cfg['look_scale_linear'],
                               self._cfg['look_scale_angular'])
        with self._lock:
            self._look = look
            self._last_cmd = time.monotonic()

    def state(self):
        with self._lock:
            stale = (time.monotonic() - self._last_cmd) > self._cfg['motion_timeout']
            look = (0.0, 0.0) if stale else self._look
            return {'mood': self._mood, 'look': look, 'moving': not stale}

    # ------------------------------------------------------------------ http
    def _serve(self, host, port):
        node = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *_):
                pass  # one line per SSE keepalive would bury the ROS log

            def _head(self, ctype, length=None):
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                if length is None:
                    self.send_header('Cache-Control', 'no-cache')
                else:
                    self.send_header('Content-Length', str(length))
                self.end_headers()

            def do_GET(self):
                if self.path.startswith('/events'):
                    return self._events()
                if self.path.startswith('/health'):
                    self._head('text/plain', 3)
                    self.wfile.write(b'ok\n')
                    return
                # Re-read every time: with colcon --symlink-install the share
                # copy IS the source file, so editing the page and hitting
                # refresh is the whole edit loop. It is 6 kB.
                try:
                    with open(node._page, 'rb') as fh:
                        body = fh.read()
                except OSError as exc:
                    self.send_error(500, 'cannot read face.html: %s' % exc)
                    return
                self._head('text/html; charset=utf-8', len(body))
                self.wfile.write(body)

            def _events(self):
                self._head('text/event-stream')
                last, beat = None, time.monotonic()
                try:
                    while True:
                        st = node.state()
                        now = time.monotonic()
                        # Only on change, plus a keepalive so a proxy or a dozed
                        # laptop cannot leave the socket half-open unnoticed.
                        if st != last or now - beat > 15.0:
                            self.wfile.write(
                                b'data: ' + json.dumps(st).encode() + b'\n\n')
                            self.wfile.flush()
                            last, beat = st, now
                        time.sleep(0.05)          # 20 Hz, same as joy_node
                except (BrokenPipeError, ConnectionResetError):
                    pass                          # browser closed the tab

        try:
            self._http = ThreadingHTTPServer((host, port), Handler)
        except OSError as exc:
            # Do not die: joy/cmd_vel keep working and `make up` reports the
            # face layer as down, which is a far clearer failure than a node
            # that vanished out of a launch.
            self.get_logger().error(
                'cannot bind %s:%d (%s). Another server on that port? '
                'Set the `port` parameter in config/teleop_params.yaml.'
                % (host, port, exc))
            self._http = None
            return
        self._http.daemon_threads = True
        threading.Thread(target=self._http.serve_forever, daemon=True).start()
        self.get_logger().info(
            'face ready: http://localhost:%d  (A=happy B=angry X=tired Y=surprised)'
            % port)

    def destroy_node(self):
        if getattr(self, '_http', None):
            self._http.shutdown()
        super().destroy_node()


def _selftest():
    cfg = {'button_happy': 0, 'button_angry': 1,
           'button_tired': 2, 'button_surprised': 3}
    assert mood_from_buttons([0, 0, 0, 0], cfg) == 'default'
    assert mood_from_buttons([1, 0, 0, 0], cfg) == 'happy'
    assert mood_from_buttons([0, 0, 0, 1], cfg) == 'surprised'
    assert mood_from_buttons([1, 1, 0, 0], cfg) == 'happy'   # priority order
    assert mood_from_buttons([], cfg) == 'default'           # short Joy array
    assert mood_from_buttons([1], {**cfg, 'button_happy': -1}) == 'default'

    assert look_from_twist(0, 0, 0, 0.6, 2.0) == (0.0, 0.0)
    assert look_from_twist(0.6, 0, 0, 0.6, 2.0) == (0.0, 1.0)     # forward = up
    assert look_from_twist(0, 0.6, 0, 0.6, 2.0) == (-1.0, 0.0)    # +y left
    assert look_from_twist(0, 0, 2.0, 0.6, 2.0) == (-1.0, 0.0)    # +z ccw = left
    assert look_from_twist(0, -0.3, 0, 0.6, 2.0) == (0.5, 0.0)    # right, half
    assert look_from_twist(0, 9, 9, 0.6, 2.0) == (-1.0, 0.0)      # clamped
    print('face_node selftest ok')


def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    import sys
    if '--selftest' in sys.argv:
        _selftest()
    else:
        main()
