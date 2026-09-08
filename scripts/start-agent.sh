#!/usr/bin/env bash
# Start the micro-ROS agent for the chassis ESP32 and bring the board out of reset.
#
# Baked into the image at /root/scripts/start-agent.sh and invoked by `make agent`
# (host -> docker exec). It lives in the image, not the bind-mounted workspace, so
# the workaround survives an empty ros2_ws and a container rebuild.
#
# Two quirks it handles (see CLAUDE.md):
#   1. DDS: everything runs on Fast-DDS + localhost unicast discovery.
#      RMW_IMPLEMENTATION and FASTRTPS_DEFAULT_PROFILES_FILE are baked as image
#      ENV, so `docker exec` already has them in this process's environment; we
#      only need to source ROS here, since docker exec runs neither the
#      entrypoint nor .zshrc and `ros2` would otherwise not be on PATH.
#   2. CH340/CH341 auto-reset: RTS is wired to EN(reset). When the agent opens the
#      serial port the kernel asserts RTS, holding the ESP32 in reset so the
#      firmware never boots. We start the agent, wait for it to open the port,
#      then drop RTS/DTR with release_rts.py so the board boots and connects.
#      (If you later disable the RTS->EN auto-reset on the board in hardware, the
#       release becomes a harmless no-op — safe to keep either way.)
#
# Usage:  start-agent.sh [DEV] [BAUD]   (defaults: /dev/esp32 115200)

DEV="${1:-/dev/esp32}"
BAUD="${2:-115200}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Refuse to become the SECOND agent on this port. Two agents each read half the
# XRCE byte stream, so neither can hold a session -- and the symptom is that
# /esp32_base vanishes from `ros2 node list`, which is byte-for-byte the symptom
# of no agent at all. That sends you to CLAUDE.md's diagnosis list, whose first
# step is "run start-agent.sh again", which would make a third one. Seen for
# real on 2026-09-03: a detached agent plus a `make agent` in another terminal.
# Note the [_] in the pattern -- without it pgrep matches this script's own
# command line and reports a phantom agent.
#
# ZOMBIES DO NOT COUNT. A dead agent that has not been reaped still shows up in
# pgrep, but holds no serial port and no DDS session. If PID 1 in the container
# does not reap (a bare `sleep`, or any CMD that ignores SIGCHLD), the corpse
# lingers forever and this guard would refuse to EVER start an agent again --
# while `ros2 node list` stays empty, which reads exactly like "no agent". Seen
# for real on 2026-09-08. `docker run --init` (now in the Makefile) reaps them,
# and this filter means the guard survives a container started without it.
EXISTING=""
for _p in $(pgrep -f 'micro_ros[_]agent'); do
  _state="$(ps -o stat= -p "$_p" 2>/dev/null | tr -d ' ')"
  case "$_state" in
    Z*|'') continue ;;          # zombie, or already gone between the two calls
  esac
  EXISTING="$EXISTING $_p"
done
EXISTING="${EXISTING# }"
if [ -n "$EXISTING" ]; then
  echo "[start-agent] REFUSING TO START: an agent is already running (PIDs: $EXISTING)."
  echo "[start-agent] If it is healthy, you need nothing -- check:  ros2 node list"
  echo "[start-agent] If it is stuck, kill it first and re-run this script:"
  echo "[start-agent]     pkill -f 'micro_ros[_]agent'"
  exit 1
fi

# docker exec does not source ROS; do it here so `ros2` is on PATH.
source /opt/ros/humble/setup.bash
[ -f /opt/uros_ws/install/local_setup.bash ] && source /opt/uros_ws/install/local_setup.bash

echo "[start-agent] launching micro-ROS agent on $DEV @ $BAUD (Fast-DDS, localhost unicast)"
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b "$BAUD" &
AGENT_PID=$!

# Forward Ctrl-C / docker stop to the agent so it shuts down cleanly.
trap 'kill "$AGENT_PID" 2>/dev/null' INT TERM

# Wait for the agent to open the port (which asserts RTS and holds the board in
# reset), then release RTS so the ESP32 boots into the now-listening agent.
sleep 2
python3 "$HERE/release_rts.py" "$DEV" "$BAUD" || \
  echo "[start-agent] WARNING: could not release RTS on $DEV (board may stay in reset)"

echo "[start-agent] agent PID=$AGENT_PID — check with:  ros2 node list"
wait "$AGENT_PID"
