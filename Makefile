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

.PHONY: help build sim flight-test stop shell shell-px4 logs ps clean clean-all

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
	@echo "  ║    make flight-test Fly takeoff-hover-land    ║"
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

# Build common_* packages inside the container and fly the Phase 1 test:
# arm → offboard → takeoff to 2 m → hover → land → disarm.
# PX4's EKF2 needs ~30-60 s after `make sim` before preflight checks pass —
# the node retries once per second until PX4 accepts, so just leave it running.
flight-test:
	@echo "==> Building common_* packages and running the offboard flight test..."
	@docker exec -it ros2-autonomy bash -c "\
		source /opt/ros/humble/setup.bash && \
		source /opt/px4_ros2_ws/install/setup.bash && \
		mkdir -p /ros2_ws_build && cd /ros2_ws_build && \
		colcon build --symlink-install --base-paths /ros2_ws/src \
			--build-base /ros2_ws_build/build \
			--install-base /ros2_ws_build/install \
			--packages-select common_control && \
		source /ros2_ws_build/install/setup.bash && \
		ros2 run common_control offboard_control_node \
			--ros-args -p takeoff_height_m:=2.0 -p hover_seconds:=5.0"

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
