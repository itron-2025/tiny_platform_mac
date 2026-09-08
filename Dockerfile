# ROS2 Humble development image for the tiny_platform_mac mecanum chassis.
#
# Deliberately a SELF-CONTAINED copy of the toolchain rather than a thin layer on
# top of the Tomcat project's image: the two robots are separate machines with
# separate wiring, and a shared base would mean a rebuild over there silently
# changes the firmware toolchain over here. Costs one extra download of the
# esp32 core; buys the freedom to move the core/library pins independently.
FROM ros:humble

# Terminal colors + non-interactive apt
ENV TERM=xterm-256color
ENV DEBIAN_FRONTEND=noninteractive

# ROS2 runtime environment, set at the IMAGE level on purpose.
# `docker exec` (used by `make attach` / `make agent`) does NOT run the
# entrypoint or source .zshrc, so a shell-only `export` would silently miss the
# micro-ROS agent. Setting these as ENV makes run / attach / agent consistent.
#
# DDS = Fast-DDS, system-wide. This is forced, not a free choice:
#   1. The baked micro-ROS agent (/opt/uros_ws) links libfastrtps and ALWAYS
#      speaks Fast-DDS, ignoring RMW_IMPLEMENTATION. If the rest of the system
#      ran CycloneDDS, the agent and the nodes would never discover each other.
#      ALL participants must share one RMW, so everything is Fast-DDS.
#   2. The container's loopback `lo` is NOT multicast-capable
#      (/sys/class/net/lo/flags = 0x9 = UP|LOOPBACK, no MULTICAST bit). DDS
#      default discovery needs multicast, so plain localhost discovery silently
#      fails. FASTRTPS_DEFAULT_PROFILES_FILE points at a profile that switches
#      Fast-DDS to UNICAST 127.0.0.1 discovery, which works without multicast.
# The profile lives in the bind-mounted project, so it is the exact file the host
# can edit. ROS_LOCALHOST_ONLY is intentionally NOT set: the unicast peer in the
# profile already pins discovery to 127.0.0.1.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/tiny_platform_mac/ros2_ws/dds/fastdds_localhost.xml

# 1. System + ROS2 dev tools. Merged into one layer, apt cache cleaned in the
#    same RUN to keep the image small.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      git \
      unzip \
      curl \
      wget \
      zsh \
      tmux \
      htop \
      nano \
      udev \
      iproute2 \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
# NOTE: rmw_cyclonedds is intentionally NOT installed -- the system standardized
# on Fast-DDS (ships with ros:humble). The baked micro-ROS agent only speaks
# Fast-DDS, so mixing in CycloneDDS would just reintroduce cross-RMW discovery
# failure.

# 2. Python packages (--no-cache-dir to avoid pip cache bloat).
#    pyserial: release_rts.py and the bring-up tools in tools/.
#    numpy: the calibration / tuning scripts. Deliberately no pandas or
#    matplotlib -- plots are emitted as hand-written SVG.
RUN pip3 install --no-cache-dir pyserial numpy

# 3. micro-ROS agent, built into the image so it is ready to run.
#    Lives in its own workspace (/opt/uros_ws); sourced from .zshrc / entrypoint.
#    Never run it bare -- use scripts/start-agent.sh (see its header).
RUN apt-get update && \
    mkdir -p /opt/uros_ws/src && cd /opt/uros_ws && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf /var/lib/apt/lists/* /root/.ros/rosdep

# 4. Claude Code CLI (official native installer, no Node.js needed).
#    Installs to /root/.local/bin/claude; PATH is set in .zshrc.
RUN curl -fsSL https://claude.ai/install.sh | bash

# 5. Zsh + oh-my-zsh + powerlevel10k + autosuggestions + syntax-highlighting.
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    chsh -s $(which zsh)

# User shell config + offline p10k gitstatus cache (avoids a network fetch on
# first prompt; the bundled binary is for linux-x86_64).
COPY dotfiles/.p10k.zsh /root/.p10k.zsh
# .zshenv is sourced before /etc/zsh/zshrc; it sets skip_global_compinit=1 to
# suppress Ubuntu's global bare `compinit` (which prompts about insecure dirs
# because umask 000 makes bind-mounted dirs world-writable). See the file.
COPY dotfiles/.zshenv /root/.zshenv
COPY dotfiles/.zshrc /root/.zshrc
COPY cachefile/gitstatus /root/.cache/gitstatus

# 6. ESP32 firmware toolchain, baked in so the container can compile AND flash
#    the sketches in firmware/ without leaving the container.
#
#    The version pins below are HARD constraints, not preferences:
#      - esp32 core MUST be 2.0.x (built on ESP-IDF v4.4). micro_ros_arduino ships
#        a precompiled libmicroros.a built with IDF v4.4; core 3.x uses IDF v5 and
#        is ABI-incompatible (and also removed ledcSetup()/ledcAttachPin(), which
#        the firmware uses). Seeing `'ledcSetup' was not declared in this scope`
#        means the CORE VERSION is wrong -- do not "fix" it by rewriting the
#        firmware's LEDC calls; that only moves the failure to link time or to a
#        crash on boot.
#      - micro_ros_arduino MUST be a -humble release (matches ROS2 Humble). It is
#        precompiled=true, so it is NOT rebuilt against the core.
#    The sketches vendor Encoder.h / Motor.h locally (double-quoted includes,
#    files live in the sketch folder), so NO external Encoder library is
#    installed on purpose: PJRC's Encoder.h re-adds an IRAM_ATTR on a class-inline
#    ISR, which fails to link with `dangerous relocation: l32r: literal placed
#    after use` on Xtensa.
#
#    This single RUN downloads ~1-2 GB (esp32 core + xtensa toolchain). Kept as
#    one layer so Docker caches it whole.
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
      | BINDIR=/usr/local/bin sh && \
    arduino-cli config init && \
    arduino-cli config add board_manager.additional_urls \
      https://espressif.github.io/arduino-esp32/package_esp32_index.json && \
    arduino-cli core update-index && \
    arduino-cli core install esp32:esp32@2.0.17 && \
    arduino-cli lib install "micro_ros_arduino@2.0.7-humble" && \
    arduino-cli cache clean

# 7. Gamepad teleop.
#    The device itself needs nothing installed here -- `make run` bind-mounts the
#    host's /dev, so the Pro Controller's evdev node is already visible inside.
#    What DOES have to be baked in is the ROS2 side: the container runs with
#    --rm, so anything apt-installed from inside a running container is gone on
#    the next `make run`.
#
#    joy, NOT joy_linux. `joy` is the SDL2-based node and reads evdev
#    (/dev/input/event*). `joy_linux` reads the legacy /dev/input/jsX node, which
#    does NOT exist for the Pro Controller on this host: hid-nintendo gets no
#    joydev binding, so `ls /dev/input/js*` finds nothing. Most ROS1-era gamepad
#    tutorials assume jsX -- on this machine that path is a dead end.
#
#    teleop_twist_joy is the stock joy -> /cmd_vel node, kept as a way to prove
#    the chain end-to-end before this project's own teleop node exists.
#
#    evtest is a diagnostic and is here on purpose: when the gamepad misbehaves
#    the first question is whether the KERNEL sees it, which ROS cannot answer.
#      evtest /dev/input/event10      # prints raw button/axis events
#
#    No camera packages (usb_cam / v4l2_camera / cv_bridge): this chassis has no
#    camera in its spec. Add them to this layer if one is fitted later.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-joy \
      ros-humble-teleop-twist-joy \
      evtest \
    && rm -rf /var/lib/apt/lists/*

# NO ROS_DOMAIN_ID HERE, AND THAT IS A MEASURED DECISION, NOT AN OMISSION.
#
# The problem it would have solved is real: the Tomcat chassis project runs its
# own container on this same host with --net=host and the same 127.0.0.1 unicast
# DDS profile. Measured 2026-09-08 from inside this container, its /joy_node,
# /joy_teleop and /camera_web_stream were fully discoverable, and /cmd_vel
# showed OUR board as a live subscriber to a topic THEIR gamepad publishes.
# Holding the deadman for that robot would have driven this one.
#
# But ROS_DOMAIN_ID CANNOT fix it here. The prebuilt micro_ros_agent links
# Fast-DDS directly instead of going through an RMW, so it never reads
# ROS_DOMAIN_ID and always creates its participants on DDS domain 0 -- the same
# reason it ignores RMW_IMPLEMENTATION. Verified: with ROS_DOMAIN_ID=7 set for
# the agent process, its topics still appeared on domain 0 and domain 7 was
# empty, and `micro_ros_agent --help` offers no domain option at all (-d is the
# discovery PORT). Setting it would therefore have been worse than useless: our
# own nodes would move to domain 7 while the board stayed on 0, and /cmd_vel
# would never reach the wheels.
#
# What this project does instead: the firmware puts its node and every topic
# under the "tiny" NAMESPACE (/tiny/base, /tiny/cmd_vel, ...), see
# MICROROS_NAMESPACE in firmware/*/config.h. Tomcat publishes bare /cmd_vel, so
# the two robots can share a host and a gamepad without either one moving when
# the other is driven.

WORKDIR /root/tiny_platform_mac

# udev rules + entrypoint + micro-ROS agent launcher.
# start-agent.sh / release_rts.py are baked into the image (not the bind-mounted
# project) so `make agent` works even before the project has any code, and
# survives a container rebuild. They work around the CH340/CH341 RTS->EN
# auto-reset quirk; see the script headers and CLAUDE.md.
COPY scripts/*.rules /root/scripts/
COPY scripts/entrypoint.sh /root/scripts/entrypoint.sh
COPY scripts/start-agent.sh /root/scripts/start-agent.sh
COPY scripts/release_rts.py /root/scripts/release_rts.py
RUN chmod +x /root/scripts/entrypoint.sh /root/scripts/start-agent.sh /root/scripts/release_rts.py

ENTRYPOINT ["/root/scripts/entrypoint.sh"]
CMD ["zsh"]
