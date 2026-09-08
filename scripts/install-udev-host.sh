#!/usr/bin/env bash
# Install this project's udev rules on the HOST. Run once per host.
#
# Rules installed:
#   esp32.rules        -> /dev/esp32                  (ESP32 USB serial)
#   peripherals.rules  -> /dev/input/pro_controller   (Bluetooth gamepad)
#
# Why on the host (and NEVER inside the container):
#   The container runs with `--privileged --net=host`, so it shares the host's
#   kernel uevent netlink socket. Running `udevadm trigger` inside the container
#   re-fires uevents for EVERY host device (GPU, input, ...), which crashes the
#   host desktop (drops to the login screen and freezes). Because `make run`
#   bind-mounts the host's /dev, symlinks created by the HOST's udev show up
#   inside the container for free.
#
# Usage:
#   scripts/install-udev-host.sh            # install the rules (asks for sudo)
#   scripts/install-udev-host.sh --detect   # just list device ids, install nothing

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_DIR=/etc/udev/rules.d

# "source basename:destination basename"
RULES=(
  "esp32.rules:99-tiny-esp32.rules"
  "peripherals.rules:99-tiny-peripherals.rules"
)

# --- List candidate devices and their udev match attributes (no root needed) ---
detect_devices() {
  local found dev props vid pid vendor model name

  echo "USB-serial devices (/dev/ttyUSB*, /dev/ttyACM*) -- for esp32.rules:"
  found=0
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    found=1
    props="$(udevadm info -q property -n "$dev" 2>/dev/null || true)"
    vid="$(printf '%s\n' "$props" | sed -n 's/^ID_VENDOR_ID=//p')"
    pid="$(printf '%s\n' "$props" | sed -n 's/^ID_MODEL_ID=//p')"
    vendor="$(printf '%s\n' "$props" | sed -n 's/^ID_VENDOR=//p')"
    model="$(printf '%s\n' "$props" | sed -n 's/^ID_MODEL=//p')"
    printf '  %-14s idVendor=="%s"  idProduct=="%s"   (%s %s)\n' \
      "$dev" "${vid:-????}" "${pid:-????}" "${vendor:-?}" "${model:-?}"
  done
  [ "$found" -eq 1 ] || echo "  (none found -- plug in the ESP32 and try again)"

  echo
  echo "Gamepads (/dev/input/event*) -- for peripherals.rules:"
  found=0
  for dev in /dev/input/event*; do
    [ -e "$dev" ] || continue
    props="$(udevadm info -q property -n "$dev" 2>/dev/null || true)"
    # udev tags every joystick-like input node with ID_INPUT_JOYSTICK=1.
    printf '%s\n' "$props" | grep -q '^ID_INPUT_JOYSTICK=1' || continue
    found=1
    name="$(udevadm info -a -n "$dev" 2>/dev/null | sed -n 's/.*ATTRS{name}=="\(.*\)"/\1/p' | head -1)"
    printf '  %-20s name=="%s"\n' "$dev" "${name:-?}"
  done
  [ "$found" -eq 1 ] || echo "  (none found -- connect the controller and try again)"
  echo
  echo "NOTE: joy_node matches on the SDL name, which for this controller is"
  echo "      \"Nintendo Switch Pro Controller\" -- NOT the evdev name above."
}

if [ "${1:-}" = "--detect" ]; then
  detect_devices
  exit 0
fi

# --- Warn about symlinks already claimed by another project's rules ----------
# The Tomcat chassis project installs 99-esp32.rules / 99-tomcat-peripherals.rules
# with the SAME symlink names. Two rule files both doing SYMLINK+= on the same
# device is legal in udev (the link is simply created once), so this is a notice,
# not an error -- but you should know the other file is there before debugging a
# symlink that "mysteriously" survives uninstalling this one.
for existing in "$RULES_DIR"/*.rules; do
  [ -e "$existing" ] || continue
  case "$(basename "$existing")" in 99-tiny-*) continue ;; esac
  if grep -qE 'SYMLINK\+="(esp32|input/pro_controller)"' "$existing" 2>/dev/null; then
    echo "NOTE: $existing already claims one of our symlinks (another project's rules)."
  fi
done

echo "Installing udev rules into $RULES_DIR (sudo required)..."
for pair in "${RULES[@]}"; do
  src="${pair%%:*}"; dst="${pair##*:}"
  sudo install -m 0644 "$SCRIPT_DIR/$src" "$RULES_DIR/$dst"
  echo "  $src -> $RULES_DIR/$dst"
done

sudo udevadm control --reload-rules

# Re-fire uevents with a PRECISE scope. A bare `udevadm trigger` would re-fire
# every input device on the host, including the keyboard and mouse currently in
# use. Two narrow triggers instead:
sudo udevadm trigger --subsystem-match=tty --attr-match=idVendor=1a86 || true
sudo udevadm trigger --subsystem-match=input --attr-match=name="Pro Controller" || true

echo
echo "Done. Current state:"
for link in /dev/esp32 /dev/input/pro_controller; do
  if [ -e "$link" ]; then
    echo "  $link -> $(readlink -f "$link")"
  else
    echo "  $link  (not present -- device not connected?)"
  fi
done
