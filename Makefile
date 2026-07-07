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

.PHONY: help build sim stop shell shell-px4 logs ps clean clean-all

help:
	@echo ""
	@echo "  ╔══════════════════════════════════════════════╗"
	@echo "  ║   GPS-Denied Drone — Quick Reference         ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  FIRST TIME ONLY                             ║"
	@echo "  ║    make build      Build Docker images        ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  DAILY WORKFLOW                              ║"
	@echo "  ║    make sim        Start PX4 SITL + ROS 2     ║"
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
