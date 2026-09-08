#!/bin/bash
# Entrypoint for the tiny_platform_mac ROS2 Humble dev container.
# Handles bind-mount file ownership, ROS2 sourcing, an optional workspace build,
# and prints the bring-up cheat sheet.
#
# NOTE: deliberately no `set -e` and no `exec` -- we must always reach the final
# chown that restores host ownership, even if the interactive shell exits
# non-zero.

PROJ=/root/tiny_platform_mac

# --- Host file ownership ---
# Container runs as root; files created in the bind-mounted project would be
# root-owned on the host (permission denied when editing from the host).
# umask 000 makes new files world-writable; we chown back to HOST_UID on exit.
umask 000
HOST_UID="${HOST_UID:-1000}"

# --- Source ROS2 ---
source /opt/ros/humble/setup.bash
# micro-ROS agent overlay (baked into the image at /opt/uros_ws)
[ -f /opt/uros_ws/install/local_setup.bash ] && source /opt/uros_ws/install/local_setup.bash

# RMW (Fast-DDS) and FASTRTPS_DEFAULT_PROFILES_FILE (localhost unicast profile)
# are set at the image level (Dockerfile ENV) so run/attach/agent stay
# consistent -- nothing to export here.

# --- Build the workspace only if it actually contains packages ---
# An empty ros2_ws/src (just .gitkeep) would make `colcon build` error out.
cd "$PROJ/ros2_ws"
if find src -mindepth 2 -maxdepth 3 \( -name package.xml -o -name CMakeLists.txt \) 2>/dev/null | grep -q .; then
  echo "Found ROS2 packages in ros2_ws/src/, running colcon build..."
  colcon build --symlink-install || echo "  colcon build failed; dropping into shell anyway."
  chown -R "$HOST_UID:$HOST_UID" "$PROJ/ros2_ws" 2>/dev/null
  source install/setup.bash
else
  echo "ros2_ws/src is empty (no packages yet) -- skipping colcon build."
fi
cd "$PROJ"

# --- Load baked-in Claude Code skills into the (persisted) config dir ---
# CLAUDE_CONFIG_DIR is bind-mounted to the host for login persistence, which
# shadows anything baked into that path in the image. So we copy the staged
# skills in on every boot (idempotent).
CLAUDE_DIR="${CLAUDE_CONFIG_DIR:-/root/.claude}"
if [ -d /opt/claude-skills ] && [ -n "$(ls -A /opt/claude-skills 2>/dev/null)" ]; then
  mkdir -p "$CLAUDE_DIR/skills"
  cp -r /opt/claude-skills/* "$CLAUDE_DIR/skills/"
  echo "Loaded Claude skills into $CLAUDE_DIR/skills/: $(ls /opt/claude-skills | tr '\n' ' ')"
fi

# --- USB device naming (/dev/esp32) is handled on the HOST, not in here ---
# DANGER: never start udevd or run `udevadm trigger`/`udevadm control` inside
# this container. It runs with `--privileged --net=host`, so it shares the
# host's kernel uevent netlink socket. `udevadm trigger` would re-fire uevents
# for EVERY host device (GPU, input, ...), which crashes the host desktop.
#
# Because `make run` bind-mounts the host's /dev, a symlink created by the
# HOST's udev appears in here automatically.
echo ""
echo "USB device names like /dev/esp32 come from the HOST's udev rules."
echo "Install them once per host with:  make install-udev   (on the host)"
if [ -e /dev/esp32 ]; then
  echo "  /dev/esp32 is present -> $(readlink -f /dev/esp32)"
else
  echo "  /dev/esp32 not found yet -- fall back to /dev/ttyUSB0."
fi

echo ""
cat <<'BANNER'
   __  _                ____  __      __  ____
  / /_(_)__  __ __     / __ \/ /___ _/ /_/ __/__  ______ _
 / __/ / _ \/ // /    / /_/ / / __ `/ __/ /_/ __ \/ ___/ /_/
/ /_/ /  __/\_, /    / ____/ / /_/ / /_/ __/ /_/ / /  /  __/
\__/_/\___//___/    /_/   /_/\__,_/\__/_/  \____/_/   \___/
BANNER
echo "========== tiny_platform_mac  --  ROS2 Humble dev container =========="
echo " Project root (bind-mounted): $PROJ"
echo ""
echo " Start the micro-ROS agent (ALWAYS via this script, never bare):"
echo "   /root/scripts/start-agent.sh /dev/esp32 115200      (or, on the host: make agent)"
echo ""
echo " Compile + flash the ESP32 (stop the agent first, it holds the serial port):"
echo "   pkill -f 'micro_ros[_]agent'"
echo "   arduino-cli compile --fqbn esp32:esp32:esp32 firmware/tiny_open"
echo "   arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/esp32 firmware/tiny_open"
echo ""
echo " Build the ROS2 workspace after adding packages to ros2_ws/src/:"
echo "   cd \$PROJ/ros2_ws && colcon build --symlink-install && source install/setup.zsh"
echo "======================================================================"
echo ""

# --- Run the main command (interactive zsh from CMD). ---
"$@"

# --- Restore project ownership to the host user on exit ---
echo "Restoring $PROJ ownership to host user (uid $HOST_UID)..."
chown -R "$HOST_UID:$HOST_UID" "$PROJ" 2>/dev/null
echo "Done."
