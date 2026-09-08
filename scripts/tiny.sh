#!/usr/bin/env bash
# One-command driver for the tiny_platform_mac stack. Called by the Makefile.
#
# The stack has four layers that must come up in a specific order, and getting
# that order wrong produces symptoms that all look identical ("nothing moves",
# "ros2 node list is empty"). This script encodes the order so nobody has to
# remember it:
#
#   1. container   -- detached, so it survives closing the terminal
#   2. micro-ROS agent  -- MUST be listening BEFORE the ESP32 boots. The
#      firmware's error_loop() has no retry: if the agent is not there when the
#      board comes up, it stays in a blink loop forever and no amount of
#      starting the agent afterwards helps. start-agent.sh handles this by
#      holding the board in reset (via the CH340's RTS line) until the agent is
#      listening, then releasing it.
#   3. teleop      -- only once telemetry is actually flowing, so a failure here
#      cannot be confused with a failure in layer 2.
#   4. face        -- the web face. Last because it is the only layer nothing
#      else depends on; it is also started SEPARATELY from the teleop launch
#      file on purpose, so that a face that will not start (stale workspace,
#      port already taken) cannot take the driving nodes down with it.
#
# Usage:  scripts/tiny.sh {up|down|status|watch|shell|logs}

set -uo pipefail

IMAGE=tiny-platform:latest
NAME=tiny-platform
PROJ_DIR=/root/tiny_platform_mac
DEV=${DEV:-/dev/esp32}
BAUD=${BAUD:-115200}
FACE_PORT=${FACE_PORT:-8088}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

R=$'\033[0m'; B=$'\033[1m'; G=$'\033[32m'; Y=$'\033[33m'; E=$'\033[31m'

ok()   { echo "  ${G}OK${R}    $*"; }
warn() { echo "  ${Y}WARN${R}  $*"; }
fail() { echo "  ${E}FAIL${R}  $*"; }

in_container() { docker exec "$NAME" bash -lc "$1" 2>/dev/null; }
ros()          { docker exec "$NAME" bash -lc "source /opt/ros/humble/setup.bash; $1" 2>/dev/null; }

container_running() { [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = "true" ]; }

# A process that is a zombie (Z) or stopped (T) still shows up in pgrep but is
# not running. Both have bitten this project; check the state, not just the PID.
proc_alive() {
    docker exec "$NAME" bash -c \
        "ps -eo stat,args | grep -E '$1' | grep -v grep | grep -vE '^[[:space:]]*[ZT]'" \
        >/dev/null 2>&1
}

topic_hz() {
    ros "timeout 6 ros2 topic hz $1 2>&1 | grep -o 'average rate: [0-9.]*' | head -1"
}

# Kill processes matching an extended regex INSIDE the container.
#
# Written the long way on purpose. `pkill -f PATTERN` matches the killing
# command's OWN command line, so it kills the shell that ran it; the usual
# `pkill -f 'joy[_]node'` bracket trick only protects the pattern itself, and
# breaks again the moment the same line mentions the name anywhere else. This
# project has been bitten by that three times (CLAUDE.md 3.7). Here the shell
# excludes its own PID and its parent's, and drops the grep, so nothing in the
# pipeline can match itself.
kill_matching() {
    local pids
    pids=$(docker exec "$NAME" bash -c \
        "ps -eo pid,args --no-headers \
         | awk -v me=\$\$ -v par=\$PPID '\$1 != me && \$1 != par' \
         | grep -E '$1' | grep -v 'grep -E' | awk '{print \$1}'" 2>/dev/null | tr '\n' ' ')
    [ -n "$pids" ] || return 0
    # shellcheck disable=SC2086 - deliberate word splitting, these are PIDs
    docker exec "$NAME" kill $pids 2>/dev/null
    sleep 3
}

start_agent()  { docker exec -d "$NAME" /root/scripts/start-agent.sh "$DEV" "$BAUD"; }
start_face()   {
    docker exec -d "$NAME" bash -lc \
        "cd $PROJ_DIR/ros2_ws && source /opt/ros/humble/setup.bash \
         && source install/setup.bash \
         && ros2 run tiny_teleop face_node --ros-args -r __ns:=/tiny \
            --params-file \$(ros2 pkg prefix tiny_teleop)/share/tiny_teleop/config/teleop_params.yaml \
         > /tmp/face.log 2>&1"
}

# Judged by whether the page ANSWERS, not by whether the process exists -- the
# same rule as every other layer here, and for the same reason: face_node keeps
# running when it cannot bind the port (deliberately, so it does not vanish out
# of the stack), and a live PID would report that as healthy.
face_ok() {
    docker exec "$NAME" curl -sf -m 2 "http://127.0.0.1:$FACE_PORT/health" \
        >/dev/null 2>&1
}
start_teleop() {
    docker exec -d "$NAME" bash -lc \
        "cd $PROJ_DIR/ros2_ws && source /opt/ros/humble/setup.bash \
         && source install/setup.bash \
         && ros2 launch tiny_teleop teleop.launch.py > /tmp/teleop.log 2>&1"
}

wait_for_topic() {   # $1 = topic, $2 = seconds
    local i
    for i in $(seq 1 "$2"); do
        [ -n "$(topic_hz "$1")" ] && return 0
        printf "."
        sleep 1
    done
    [ -n "$(topic_hz "$1")" ]
}

# Wait for ESP32 telemetry, re-releasing the CH340 RTS line every few seconds.
#
# WHY THE RETRY. start-agent.sh releases RTS once, two seconds after launching
# the agent. RTS on this board is wired to EN, so while it is asserted the ESP32
# is held in reset. Two seconds is usually enough for the agent to open the port
# -- but not on a container that has just started, where loading the agent binary
# takes longer. Release too early and the agent then opens the port, the kernel
# asserts RTS, and NOTHING ever lowers it again: the board sits in reset forever
# while the agent waits for a client. Both processes look perfectly healthy and
# `ros2 topic hz` is silent, which is indistinguishable from a dead board.
# Observed 2026-09-08 straight after a firmware flash.
#
# Releasing again is harmless when it was not needed (it is idempotent), so the
# fix is simply to keep doing it until telemetry appears.
wait_for_esp32() {   # $1 = seconds
    local i
    for i in $(seq 1 "$1"); do
        [ -n "$(topic_hz /tiny/wheel_ticks)" ] && return 0
        printf "."
        if [ $((i % 6)) -eq 0 ]; then
            docker exec "$NAME" python3 /root/scripts/release_rts.py "$DEV" "$BAUD" \
                >/dev/null 2>&1
            printf "R"
        fi
        sleep 1
    done
    [ -n "$(topic_hz /tiny/wheel_ticks)" ]
}

# ---------------------------------------------------------------------------
cmd_up() {
    echo "${B}1/4  container${R}"
    if container_running; then
        ok "$NAME already running"
    else
        if [ ! -e "$DEV" ]; then
            fail "$DEV not found. Is the ESP32 plugged in? Did you run 'make install-udev'?"
            echo "        Falling back is fine: DEV=/dev/ttyUSB0 make up"
            return 1
        fi
        mkdir -p "$HOME/.claude-docker"
        docker run -d --rm --init --privileged --net=host --name "$NAME" \
            --ulimit nofile=1024:524288 \
            -v /dev:/dev \
            --mount "type=bind,source=$HERE,target=$PROJ_DIR" \
            --mount "type=bind,source=$HOME/.claude-docker,target=/root/.claude-persist" \
            -e CLAUDE_CONFIG_DIR=/root/.claude-persist \
            -e "HOST_UID=$(id -u)" \
            --entrypoint sleep "$IMAGE" infinity >/dev/null || { fail "docker run"; return 1; }
        ok "started (detached)"
    fi

    # A LIVE PROCESS IS NOT A WORKING ONE. Checking only for a PID is how this
    # script previously reported three green lines while the robot was deaf: the
    # agent can be up but not have a session, and joy_node can be up holding a
    # device that no longer exists. Each layer is therefore judged by its TOPIC,
    # and restarted if the process is there but silent.
    echo "${B}2/4  micro-ROS agent${R}"
    if proc_alive 'micro_ros_agent serial' && [ -n "$(topic_hz /tiny/wheel_ticks)" ]; then
        ok "already running, telemetry flowing"
    else
        if proc_alive 'micro_ros_agent serial'; then
            warn "agent process is up but /tiny/wheel_ticks is silent -> restarting it"
            kill_matching 'micro_ros_agent'
        fi
        start_agent
        printf "        waiting for the ESP32 to hand shake"
        if wait_for_esp32 30; then
            echo; ok "telemetry flowing"
        else
            echo
            fail "no /tiny/wheel_ticks. See CLAUDE.md section 7 before retrying --"
            echo "        starting a SECOND agent looks exactly like having none."
            return 1
        fi
    fi

    echo "${B}3/4  teleop${R}"
    if proc_alive 'joy_teleop' && [ -n "$(topic_hz /tiny/joy)" ]; then
        ok "already running, gamepad live"
    else
        if proc_alive 'joy_teleop'; then
            # The usual cause: the controller went to sleep and reconnected AFTER
            # joy_node started. SDL enumerated the old device node and retries
            # against it forever ("Unable to open joystick 0: Couldn't read
            # device info"), so the process looks healthy and publishes nothing.
            # Restarting it makes SDL re-enumerate.
            warn "joy_node is up but /tiny/joy is silent (controller reconnected"
            echo "        after it started?) -> restarting teleop"
            kill_matching 'joy_node|joy_teleop|ros2 launch tiny_teleop'
        fi
        start_teleop
        printf "        waiting for the gamepad"
        if wait_for_topic /tiny/joy 15; then
            echo; ok "gamepad live"
        else
            echo
            warn "no /tiny/joy. Wake the controller (press Home) and re-run 'make up'."
            echo "        If it stays silent: bluetoothctl info <MAC>, then 'make watch'."
        fi
    fi

    echo "${B}4/4  face${R}"
    if face_ok; then
        ok "serving on http://localhost:$FACE_PORT"
    else
        proc_alive 'face_node' && kill_matching 'face_node'
        start_face
        sleep 2
        if face_ok; then
            ok "serving on http://localhost:$FACE_PORT"
        else
            warn "face not answering on :$FACE_PORT -- 'make logs-face' says why."
            echo "        Usually one of two things: the workspace predates"
            echo "        face_node ('make rebuild-ws'), or something else already"
            echo "        holds the port (change it in config/teleop_params.yaml)."
        fi
    fi

    echo
    cmd_status
}

cmd_status() {
    echo "${B}status${R}"
    container_running && ok "container $NAME" || { fail "container not running -- 'make up'"; return 1; }
    proc_alive 'micro_ros_agent serial' && ok "micro-ROS agent" || fail "agent not running"
    proc_alive 'joy_node'   && ok "joy_node"   || warn "joy_node not running"
    proc_alive 'joy_teleop' && ok "joy_teleop" || warn "joy_teleop not running"
    face_ok && ok "face  http://localhost:$FACE_PORT" || warn "face not serving on :$FACE_PORT"

    local t j c
    t=$(topic_hz /tiny/wheel_ticks); j=$(topic_hz /tiny/joy); c=$(topic_hz /tiny/cmd_vel)
    [ -n "$t" ] && ok "ESP32 telemetry   ${t#average rate: } Hz" || fail "no /tiny/wheel_ticks (board or agent)"
    [ -n "$j" ] && ok "gamepad           ${j#average rate: } Hz" || fail "no /tiny/joy (controller not connected?)"
    # Silence here is the NORMAL idle state, not a fault: joy_teleop stops
    # publishing once the sticks have been centred for a second, so that it does
    # not fight the calibration tools for cmd_vel (see CLAUDE.md 3.15).
    if [ -n "$c" ]; then
        ok "commands out      ${c#average rate: } Hz"
    elif proc_alive 'joy_teleop'; then
        ok "commands out      idle (sticks centred -- publishes on demand)"
    else
        warn "no /tiny/cmd_vel and no joy_teleop"
    fi

    # The other project's stack shares this host, this gamepad and this DDS
    # domain. Our /tiny namespace keeps it from driving this robot, but running
    # both at once still means one controller feeding two teleop nodes.
    if [ "$(docker inspect -f '{{.State.Running}}' ros2-humble-dev 2>/dev/null)" = "true" ]; then
        warn "the Tomcat container (ros2-humble-dev) is also running."
        echo "        Harmless -- our /tiny namespace isolates us -- but it is"
        echo "        reading the same gamepad. 'docker stop ros2-humble-dev' to be sure."
    fi
    echo
    echo "  drive it:   push the sticks (DIRECT mode, no button to hold)"
    echo "  see input:  make watch"
    echo "  its face:   http://localhost:$FACE_PORT   (A happy / B angry / X tired / Y surprised)"
}

cmd_down()  { docker stop "$NAME" >/dev/null 2>&1 && echo "stopped" || echo "was not running"; }
cmd_watch() { docker exec -it "$NAME" bash -lc \
                "source /opt/ros/humble/setup.bash && python3 $PROJ_DIR/tools/joy_watch.py"; }
cmd_shell() { docker exec -it "$NAME" /bin/zsh; }
cmd_logs()  { docker exec "$NAME" bash -c 'tail -40 /tmp/teleop.log' 2>/dev/null || echo "no log yet"; }
cmd_logs_face() { docker exec "$NAME" bash -c 'tail -40 /tmp/face.log' 2>/dev/null || echo "no log yet"; }

case "${1:-}" in
    up)     cmd_up ;;
    down)   cmd_down ;;
    status) cmd_status ;;
    watch)  cmd_watch ;;
    shell)  cmd_shell ;;
    logs)   cmd_logs ;;
    logs-face) cmd_logs_face ;;
    *) echo "usage: $0 {up|down|status|watch|shell|logs}"; exit 2 ;;
esac
