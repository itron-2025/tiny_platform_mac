IMAGE_NAME     = tiny-platform:latest
CONTAINER_NAME = tiny-platform
PROJ_DIR       = /root/tiny_platform_mac

# ---------------------------------------------------------------------------
#  EVERYDAY USE -- these four are all you normally need.
#
#      make up       start everything, in the right order, and report
#                    (prints the URL of the robot's face at the end)
#      make watch    live view of gamepad -> command -> wheels
#      make status   check each layer
#      make down     stop
#
#  The ordering matters and `up` encodes it: the micro-ROS agent has to be
#  listening BEFORE the ESP32 boots, because the firmware's error_loop() never
#  retries. Starting things by hand in the wrong order gives you a robot that
#  looks identical to a broken one.
# ---------------------------------------------------------------------------
up:
	@scripts/tiny.sh up

down:
	@scripts/tiny.sh down

status:
	@scripts/tiny.sh status

watch:
	@scripts/tiny.sh watch

shell:
	@scripts/tiny.sh shell

logs:
	@scripts/tiny.sh logs

# The face node logs separately -- it is started outside the teleop launch so a
# face that will not come up cannot take the driving nodes with it.
logs-face:
	@scripts/tiny.sh logs-face

# Rebuild the ROS2 workspace inside the running container. Needed after editing
# anything under ros2_ws/src/ -- including config/teleop_params.yaml, which is
# INSTALLED into install/ rather than read from src/, so editing it alone does
# nothing until this runs.
rebuild-ws:
	docker exec $(CONTAINER_NAME) bash -lc \
		'cd $(PROJ_DIR)/ros2_ws && source /opt/ros/humble/setup.bash \
		 && colcon build --symlink-install \
		 && chown -R $(shell id -u):$(shell id -u) $(PROJ_DIR)'
	@echo ""
	@echo "Rebuilt. Restart the nodes to pick it up:  make down && make up"

# Default: build the image then run the container.
all: build run

build:
	docker build -t $(IMAGE_NAME) .

# The WHOLE project is bind-mounted (not just ros2_ws, as the Tomcat project
# does): the firmware lives at firmware/ in the project root and the container's
# arduino-cli has to see it to compile and flash.
run:
	# xhost +local:root  # grant X11 to root (only needed for rviz2; else leave commented)

	mkdir -p $(HOME)/.claude-docker

	docker run -it --rm \
		--init \
		--privileged \
		--net=host \
		--name $(CONTAINER_NAME) \
		--ulimit nofile=1024:524288 \
		-v /dev:/dev \
		--mount type=bind,source=$(shell pwd),target=$(PROJ_DIR) \
		--mount type=bind,source=$(HOME)/.claude-docker,target=/root/.claude-persist \
		-e CLAUDE_CONFIG_DIR=/root/.claude-persist \
		-e HOST_UID=$(shell id -u) \
		$(IMAGE_NAME) /bin/zsh

	# --init runs tini as PID 1, which REAPS orphaned children. Without it a
	# crashed micro-ROS agent stays a zombie forever, and start-agent.sh's
	# "is one already running?" guard then refuses to start a replacement while
	# `ros2 node list` reads empty -- a failure that looks exactly like "no
	# agent at all". (start-agent.sh also filters zombies now; this is the belt
	# to that suspenders.) It also makes Ctrl-C behave.
	#
	# -v /dev:/dev is a TRUE bind mount of the host's /dev (not the private
	# devtmpfs that --privileged gives the container by default). Without it the
	# /dev/esp32 symlink created by the host's udev never appears inside the
	# container, and USB hot-plug does not sync either (the host re-enumerates
	# /dev/ttyUSB0 but the container never sees the new node).
	#
	# Login persistence: CLAUDE_CONFIG_DIR (holds .credentials.json) points at
	# /root/.claude-persist, bind-mounted to $(HOME)/.claude-docker on the host.
	# Run `claude` once inside the container to log in; the token stays on the
	# host, so even with --rm the next container needs no re-login.

	# X11 forwarding for GUIs (rviz2) -- uncomment these and the xhost lines above.
	# -e DISPLAY=$$DISPLAY \
	# -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	# -v $(HOME)/.Xauthority:/root/.Xauthority:rw \
	# -e XAUTHORITY=/root/.Xauthority \
	# -e QT_X11_NO_MITSHM=1 \

	# xhost -local:root  # revoke X11 access after closing the GUI

attach:
	-docker exec -it $(CONTAINER_NAME) /bin/zsh

# Start the micro-ROS agent against the ESP32 inside the running container.
# Override device/baud if needed:  make agent DEV=/dev/ttyUSB0 BAUD=115200
#
# Uses the baked start-agent.sh, which starts the agent AND releases the
# CH340/CH341 RTS line (wired to EN/reset) so the board boots instead of being
# held in reset. NEVER run `ros2 run micro_ros_agent` bare -- see the script
# header for the two reasons why.
DEV  ?= /dev/esp32
BAUD ?= 115200
agent:
	-docker exec -it $(CONTAINER_NAME) \
		/root/scripts/start-agent.sh $(DEV) $(BAUD)

# --- Firmware: compile / flash from the HOST via the running container. -------
# SKETCH selects which sketch under firmware/ to act on.
#   make flash SKETCH=tiny_open
# The agent holds the serial port, so it is stopped first and NOT restarted --
# start it again with `make agent` once you are done flashing.
SKETCH ?= tiny_pid
FQBN   ?= esp32:esp32:esp32

compile:
	docker exec -it $(CONTAINER_NAME) \
		arduino-cli compile --fqbn $(FQBN) $(PROJ_DIR)/firmware/$(SKETCH)

flash:
	-docker exec -it $(CONTAINER_NAME) pkill -f 'micro_ros[_]agent'
	docker exec -it $(CONTAINER_NAME) \
		arduino-cli compile --fqbn $(FQBN) $(PROJ_DIR)/firmware/$(SKETCH)
	docker exec -it $(CONTAINER_NAME) \
		arduino-cli upload --fqbn $(FQBN) -p $(DEV) $(PROJ_DIR)/firmware/$(SKETCH)
	@echo ""
	@echo "Flashed $(SKETCH). Restart the agent with:  make agent"

stop:
	-docker stop $(CONTAINER_NAME)
	-docker rm $(CONTAINER_NAME)

clean: stop
	-docker rmi $(IMAGE_NAME)

# Install this project's udev rules on THIS host. Run once per host.
# This runs on the HOST on purpose -- never inside the container (see the script
# header and CLAUDE.md for why doing it in-container crashes the desktop).
install-udev:
	scripts/install-udev-host.sh

# List USB-serial / gamepad devices and their udev match attributes.
detect:
	scripts/install-udev-host.sh --detect

.PHONY: all up down status watch shell logs logs-face rebuild-ws build run attach agent compile flash stop clean install-udev detect
