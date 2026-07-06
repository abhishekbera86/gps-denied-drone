# =============================================================================
# Makefile — GPS-Denied Autonomous Drone Stack
# =============================================================================
# Run from the project root:  px4_docker_ws/
#
# NORMAL WORKFLOW (2 commands):
#   make sim       → launch PX4 SITL + all Aerostack2 nodes (world up)
#   make mission   → run the autonomous GPS-denied mission
#   make stop      → shut everything down
#
# FIRST TIME ONLY:
#   make build     → build Docker images (~5-10 min)
# =============================================================================

include .env
export

.PHONY: help build build-px4 build-as2 build-vio build-hw \
        sim stop mission view \
        vio hw \
        shell logs ps health \
        clean clean-all

# ── Default: show help ───────────────────────────────────────────────────────
help:
	@echo ""
	@echo "  ╔══════════════════════════════════════════════╗"
	@echo "  ║   GPS-Denied Drone — Quick Reference         ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  FIRST TIME ONLY                             ║"
	@echo "  ║    make build      Build all Docker images   ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  DAILY WORKFLOW                              ║"
	@echo "  ║    make sim        Launch simulation world   ║"
	@echo "  ║    make view       Open live terminal viewer ║"
	@echo "  ║    make mission    Run autonomous mission    ║"
	@echo "  ║    make stop       Stop everything           ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  WITH VIO (OpenVINS GPS-denied)              ║"
	@echo "  ║    make vio        Launch sim + VIO stack    ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  DEBUGGING                                   ║"
	@echo "  ║    make shell      bash in AS2 container     ║"
	@echo "  ║    make logs       tail all container logs   ║"
	@echo "  ║    make health     run health check          ║"
	@echo "  ║    make ps         show container status     ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  HARDWARE (on Orange Pi 5)                   ║"
	@echo "  ║    make hw         start hardware stack      ║"
	@echo "  ╚══════════════════════════════════════════════╝"
	@echo ""

# =============================================================================
# BUILD (one-time)
# =============================================================================
build:
	@echo "==> Building Docker image (aerostack2/nightly-humble base + extras)..."
	@echo "    First build: ~5-10 min (MicroXRCE agent + as2_platform_pixhawk from source)"
	@echo "    Subsequent builds: instant (Docker layer cache)"
	DOCKER_BUILDKIT=1 docker compose build aerostack2

build-as2:
	DOCKER_BUILDKIT=1 docker compose build aerostack2

build-vio:
	DOCKER_BUILDKIT=1 docker compose build openvins

build-hw:
	@echo "==> Building ARM64 hardware image (run on Orange Pi 5 directly)"
	DOCKER_BUILDKIT=1 docker compose build hw_stack

# =============================================================================
# SIMULATION — main workflow target
# =============================================================================
# TWO-TERMINAL WORKFLOW:
#   Terminal 1:  make sim      (stays in foreground — shows all ROS 2 node logs)
#   Terminal 2:  make mission  (run the mission once sim is ready)
#
# What it does internally:
#   1. docker compose up         → starts ONE aerostack2 container
#   2. colcon build (inside)     → registers quad_core + quad_sim with ROS 2
#   3. ros2 launch quad_sim sim.launch.py  (THE actual ROS 2 launch file)
#      └ as2_platform_multirotor_simulator  (no PX4, no Gazebo, no GPU)
#      └ as2_state_estimator (ground_truth)
#      └ as2_motion_controller
#      └ as2_behaviors_motion
#
sim:
	@echo ""
	@echo "==> Starting aerostack2 container..."
	@docker compose --profile sim up -d
	@echo ""
	@echo "==> Launching simulation world (this terminal stays live)"
	@echo "    Open a NEW terminal and run: make mission"
	@echo ""
	docker exec -it aerostack2 bash /scripts/launch_sim.sh

# =============================================================================
# VIO SIMULATION — with OpenVINS GPS-denied estimation
# =============================================================================
vio:
	@echo ""
	@echo "==> [1/2] Starting containers (PX4 SITL + Aerostack2 + OpenVINS)..."
	@docker compose --profile sim --profile vio up -d
	@echo ""
	@echo "==> [2/2] Launching simulation world with VIO..."
	@docker exec aerostack2 bash /scripts/launch_sim.sh
	@echo ""

# =============================================================================
# MISSION — run the autonomous flight mission
# =============================================================================
mission:
	@echo "==> Running GPS-denied autonomous mission..."
	@docker exec -it aerostack2 bash -c \
		"source /opt/ros/humble/setup.bash && \
		 python3 /ros2_ws/src/quad_core/mission.py"

# =============================================================================
# VIEW — launch the alphanumeric terminal dashboard
# =============================================================================
view:
	@echo "==> Launching Aerostack2 terminal dashboard..."
	@docker exec -it aerostack2 bash -c \
		"source /opt/ros/humble/setup.bash && \
		 source /root/aerostack2_ws/install/setup.bash && \
		 ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node --ros-args -r __ns:=/drone0"

# =============================================================================
# STOP — shut down everything cleanly
# =============================================================================
stop:
	@echo "==> Stopping simulation world..."
	@docker exec aerostack2 bash -c \
		"tmux kill-session -t sim 2>/dev/null || true" 2>/dev/null || true
	@docker compose --profile sim --profile vio --profile hw down
	@echo "  ✓ All containers stopped."

# =============================================================================
# HARDWARE (Orange Pi 5 — run ON the companion computer)
# =============================================================================
# What it does:
#   1. docker compose up  → starts hw_stack container
#   2. colcon build       → builds packages inside container
#   3. ros2 launch quad_real hw.launch.py  ← THE actual ROS 2 launch file
#      (quad_real/launch/hw.launch.py — RealSense + AS2 serial Pixhawk bridge)
#
hw:
	@echo "==> Starting hardware stack (run this ON the Orange Pi 5)..."
	@docker compose --profile hw up -d
	@docker exec -it hw_stack bash /scripts/launch_hw.sh

# =============================================================================
# DEBUGGING
# =============================================================================

# Open bash inside the aerostack2 container
shell:
	docker exec -it aerostack2 bash

# Shorthand shells for other containers
shell-px4:
	docker exec -it px4_sitl bash

shell-vio:
	docker exec -it openvins bash

# Tail logs from all running containers
logs:
	docker compose logs -f

# Show container status
ps:
	docker compose ps

# Run the health check script
health:
	@bash scripts/health_check.sh

# =============================================================================
# MAINTENANCE
# =============================================================================
clean:
	@docker compose --profile sim --profile vio --profile hw down --rmi local 2>/dev/null || true
	@echo "  ✓ Local images removed. Remote base images kept."

clean-all:
	@echo "WARNING: removes all images and the ccache volume."
	@echo "Next build re-downloads everything (~5-10 min)."
	@docker compose --profile sim --profile vio --profile hw down \
		--rmi all --volumes --remove-orphans 2>/dev/null || true
