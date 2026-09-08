# Initialize the shell under a SECURE umask (see the umask 000 note near the end
# for the full rationale). oh-my-zsh creates ~/.oh-my-zsh/cache/completions during
# startup; under umask 000 that dir is world-writable (777), which makes every
# `compinit` run at startup — Ubuntu's global one AND ROS2's argcomplete hooks
# (ros2cli / rosidl_cli / ament_index) — flag it as an "insecure directory" and
# block with a "[y] or abort [n]?" prompt. Running init under 022 keeps those dirs
# secure (755); we switch back to umask 000 at the very end, after all compinit
# calls are done, so interactive work still produces host-editable files.
umask 022

# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
  source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

# Claude Code CLI (native installer puts the binary in ~/.local/bin)
export PATH="$HOME/.local/bin:$PATH"

# Path to your oh-my-zsh installation.
export ZSH="$HOME/.oh-my-zsh"

# Theme
ZSH_THEME="powerlevel10k/powerlevel10k"

# Plugins
plugins=(git zsh-autosuggestions zsh-syntax-highlighting)

# Skip oh-my-zsh's insecure-directory check. This dev container intentionally
# uses `umask 000` (see below) so the host can edit bind-mounted files, which
# makes oh-my-zsh's own cache/completions dirs world-writable (777). That is
# expected here (single-user root container), so disable the compfix warning.
ZSH_DISABLE_COMPFIX=true

source $ZSH/oh-my-zsh.sh

# To customize prompt, run `p10k configure` or edit ~/.p10k.zsh.
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh

# === ROS2 setup ===
# Underlay: the base ROS2 Humble install.
source /opt/ros/humble/setup.zsh
# Overlay: micro-ROS agent, baked into the image at /opt/uros_ws (see Dockerfile).
[ -f /opt/uros_ws/install/local_setup.zsh ] && source /opt/uros_ws/install/local_setup.zsh
# Overlay: project workspace (only exists after `colcon build`).
# The whole project is bind-mounted at /root/tiny_platform_mac (Makefile `run`),
# because the ESP32 firmware lives at firmware/ in the project root and the
# container's arduino-cli has to see it -- so the workspace is one level deeper
# than the usual ~/ros2_ws.
export TP_PROJ=/root/tiny_platform_mac
[ -f $TP_PROJ/ros2_ws/install/setup.zsh ] && source $TP_PROJ/ros2_ws/install/setup.zsh

# RMW (Fast-DDS) and FASTRTPS_DEFAULT_PROFILES_FILE (localhost unicast discovery
# profile) are set at the image level (Dockerfile ENV), inherited by every shell
# and by `docker exec` — no export needed here. See the Dockerfile ENV comment.

# === Host file ownership fix ===
# Container runs as root, so files created in the bind-mounted project end up
# root-owned on the host (causing permission denied when editing from the host).
# umask 000: new files are world-writable, so the host can edit them live.
# NOTE: this is phase 2 of the umask dance — init ran under umask 022 (top of file)
# so compinit didn't choke on world-writable cache dirs. Now that all compinit
# calls are done, relax to 000 for the rest of the interactive session.
# colcon wrapper: after every build, chown the workspace back to the host user
# (HOST_UID, defaults to 1000) so build artifacts stay host-editable too.
umask 000
colcon() {
  command colcon "$@"
  local ret=$?
  chown -R "${HOST_UID:-1000}:${HOST_UID:-1000}" "${TP_PROJ:-/root/tiny_platform_mac}" 2>/dev/null
  return $ret
}
