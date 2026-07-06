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
        sim stop mission \
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
	@echo "==> Building Docker images..."
	@echo "    First build: ~5-10 min (downloads prebuilt base images)"
	@echo "    Subsequent builds: instant (Docker layer cache)"
	DOCKER_BUILDKIT=1 docker compose build

build-px4:
	DOCKER_BUILDKIT=1 docker compose build px4_sitl

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
#
# Starts PX4 SITL + Aerostack2 containers, then waits for PX4 to boot
# and launches all AS2 nodes automatically. One command. No manual steps.
#
sim:
	@echo ""
	@echo "==> [1/2] Starting containers (PX4 SITL + Aerostack2)..."
	@docker compose --profile sim up -d
	@echo ""
	@echo "==> [2/2] Launching simulation world (waiting for PX4 + starting AS2 nodes)..."
	@echo "    This takes ~60s on first start."
	@echo ""
	@docker exec aerostack2 bash /scripts/launch_sim.sh
	@echo ""

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
hw:
	@echo "==> Starting hardware stack (run this ON the Orange Pi 5)..."
	@docker compose --profile hw up -d
	@docker exec hw_stack bash /ros2_ws/src/quad_real/launch_as2.bash
	@echo ""

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
