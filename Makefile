# =============================================================================
# Makefile — GPS-Denied Autonomous Drone Stack (v3 — PX4-native ROS 2)
# =============================================================================
# Run from the project root: px4_docker_ws/
#
# PHASE 0/1 WORKFLOW:
#   make build     First time only — builds px4-sim + ros2-autonomy images
#   make sim       Starts PX4 SITL (Gazebo Harmonic, headless) + ROS 2 bridge
#   make shell     Bash inside ros2-autonomy (ros2 topic list, etc.)
#   make stop      Shuts everything down
#
# Targets are added here only as the matching container/package actually
# exists — no aspirational targets pointing at services that aren't built.
# =============================================================================

include .env
export

.PHONY: help build sim sim-gui flight-test mission stop shell shell-px4 logs ps clean clean-all

# Mission flown by `make mission` — square or survey (see common_missions).
MISSION ?= square

help:
	@echo ""
	@echo "  ╔══════════════════════════════════════════════╗"
	@echo "  ║   GPS-Denied Drone — Quick Reference         ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  FIRST TIME ONLY                             ║"
	@echo "  ║    make build      Build Docker images        ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  DAILY WORKFLOW                              ║"
	@echo "  ║    make sim         Start PX4 SITL + ROS 2    ║"
	@echo "  ║    make sim-gui     Same, with Gazebo GUI     ║"
	@echo "  ║    make flight-test Fly takeoff-hover-land    ║"
	@echo "  ║    make mission     Fly MISSION=square|survey ║"
	@echo "  ║    make shell      Shell into ros2-autonomy   ║"
	@echo "  ║    make shell-px4  Shell into px4-sim          ║"
	@echo "  ║    make logs       Tail all container logs    ║"
	@echo "  ║    make ps         Show container status      ║"
	@echo "  ║    make stop       Stop everything            ║"
	@echo "  ╚══════════════════════════════════════════════╝"
	@echo ""

# =============================================================================
# BUILD (one-time, or after Dockerfile/version changes)
# =============================================================================
build:
	DOCKER_BUILDKIT=1 docker compose --profile sim build

# =============================================================================
# SIMULATION
# =============================================================================
sim:
	@echo "==> Starting px4-sim (PX4 v${PX4_VERSION} SITL + Gazebo Harmonic, headless)..."
	@docker compose --profile sim up -d
	@echo ""
	@echo "  ✓ px4-sim and ros2-autonomy containers are up."
	@echo "    Check PX4 boot log:   make logs"
	@echo "    Check ROS 2 topics:   make shell   (then: ros2 topic list)"
	@echo ""

# Same as `make sim` but pops the Gazebo Harmonic GUI on the host desktop so
# you can WATCH the drone fly. Layers docker-compose.gui.yml (HEADLESS=0 +
# X11/DRI passthrough). Needs an X session (DISPLAY set). If the Gazebo window
# is black or gz crashes on the GPU, retry with software rendering:
#   GZ_SW_RENDER=1 make sim-gui
sim-gui:
	@echo "==> Starting px4-sim WITH Gazebo GUI (world=${PX4_GZ_WORLD}, DISPLAY=$${DISPLAY:-:0})..."
	@xhost +local:root >/dev/null 2>&1 || echo "  ! xhost not available — GUI may be denied X access"
	@docker compose -f docker-compose.yml -f docker-compose.gui.yml --profile sim up -d
	@echo ""
	@echo "  ✓ Containers up. The Gazebo Harmonic window should open shortly."
	@echo "    (First boot takes ~20-40 s. Watch it:   make logs)"
	@echo "    Then, in another terminal:   make flight-test   or   make mission"
	@echo ""

# Build the workspace and fly the Phase 1 test through the sim_bringup layer:
# arm → offboard → takeoff to 2 m → hover → land → disarm.
# PX4's EKF2 needs ~30-60 s after `make sim` before preflight checks pass —
# the node retries once per second until PX4 accepts, so just leave it running.
# Params come from sim_bringup/config/sim_params.yaml (not inline -p anymore).
flight-test:
	@echo "==> Building workspace and running the offboard flight test (via sim_bringup)..."
	@docker exec -it ros2-autonomy bash -c "\
		source /opt/ros/humble/setup.bash && \
		source /opt/px4_ros2_ws/install/setup.bash && \
		mkdir -p /ros2_ws_build && cd /ros2_ws_build && \
		colcon build --symlink-install --base-paths /ros2_ws/src \
			--build-base /ros2_ws_build/build \
			--install-base /ros2_ws_build/install \
			--packages-up-to sim_bringup && \
		source /ros2_ws_build/install/setup.bash && \
		ros2 launch sim_bringup sim.launch.py action:=hover"

# Fly a named waypoint mission (Phase 2): make mission MISSION=square|survey.
# Same build-then-run flow as flight-test, through the sim_bringup entry point;
# the mission takes off, flies its waypoint sequence, returns and lands.
mission:
	@echo "==> Building workspace and flying the '$(MISSION)' mission (via sim_bringup)..."
	@docker exec -it ros2-autonomy bash -c "\
		source /opt/ros/humble/setup.bash && \
		source /opt/px4_ros2_ws/install/setup.bash && \
		mkdir -p /ros2_ws_build && cd /ros2_ws_build && \
		colcon build --symlink-install --base-paths /ros2_ws/src \
			--build-base /ros2_ws_build/build \
			--install-base /ros2_ws_build/install \
			--packages-up-to sim_bringup && \
		source /ros2_ws_build/install/setup.bash && \
		ros2 launch sim_bringup sim.launch.py action:=mission mission:=$(MISSION)"

stop:
	@docker compose --profile sim down
	@echo "  ✓ All containers stopped."

# =============================================================================
# DEBUGGING
# =============================================================================
shell:
	docker exec -it ros2-autonomy bash

shell-px4:
	docker exec -it px4-sim bash

logs:
	docker compose logs -f

ps:
	docker compose ps

# =============================================================================
# MAINTENANCE
# =============================================================================
clean:
	@docker compose --profile sim down --rmi local 2>/dev/null || true
	@echo "  ✓ Local images removed."

clean-all:
	@echo "WARNING: removes all images and the colcon build cache volume."
	@docker compose --profile sim down --rmi all --volumes --remove-orphans 2>/dev/null || true
