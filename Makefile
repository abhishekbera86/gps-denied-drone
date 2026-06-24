# =============================================================
# Makefile — GPS-Denied Autonomous Drone Stack
# =============================================================
# Simple, documented convenience commands.
# Always run from the project root (px4_docker_ws/).
#
# Usage:
#   make help          → show all commands
#   make build         → build all Docker images
#   make sim-up        → start simulation stack
#   make sim-down      → stop everything
# =============================================================

# Load .env so make can use the variables too
include .env
export

.PHONY: help build sim-up sim-down vio-up hw-up \
        px4-shell ros2-shell openvins-shell \
        px4-start health logs clean

# ── Default target ────────────────────────────────────────────
help:
	@echo ""
	@echo "  GPS-Denied Autonomous Drone — Make Targets"
	@echo "  ─────────────────────────────────────────────────"
	@echo ""
	@echo "  SETUP"
	@echo "    setup          Run host prerequisite installer"
	@echo "    build          Build all Docker images (takes ~35 min first time)"
	@echo "    build-px4      Build only the px4 image"
	@echo "    build-ros2     Build only the ros2 image"
	@echo ""
	@echo "  SIMULATION (Phase 1-4)"
	@echo "    sim-up         Start simulation stack (px4 + ros2 containers)"
	@echo "    vio-up         Start sim + OpenVINS (Phase 3+)"
	@echo "    sim-down       Stop and remove all containers"
	@echo "    px4-start      Start PX4 SITL inside the px4 container"
	@echo ""
	@echo "  SHELLS"
	@echo "    px4-shell      Open bash inside the px4 container"
	@echo "    ros2-shell     Open bash inside the ros2 container"
	@echo "    openvins-shell Open bash inside the openvins container"
	@echo ""
	@echo "  HARDWARE (Phase 5 — run on Orange Pi)"
	@echo "    hw-up          Start hardware stack (realsense container)"
	@echo ""
	@echo "  MONITORING"
	@echo "    health         Run health check (containers + topics)"
	@echo "    logs           Tail logs from all running containers"
	@echo "    ps             Show container status"
	@echo ""
	@echo "  MAINTENANCE"
	@echo "    clean          Stop containers + remove images (keeps volumes)"
	@echo "    clean-all      Stop containers + remove images + volumes"
	@echo "    git-init       Initialize git repo and create first commit"
	@echo ""

# ── Setup ─────────────────────────────────────────────────────
setup:
	@echo "==> Running host setup..."
	@bash scripts/setup_host.sh

# ── Build ─────────────────────────────────────────────────────
# Uses BuildKit for faster, cached layer builds
build:
	@echo "==> Building all Docker images..."
	@echo "    NOTE: First build takes ~35 minutes (PX4 compile + ROS2 workspace)"
	@echo "    Subsequent builds are instant due to Docker layer cache."
	@echo ""
	DOCKER_BUILDKIT=1 docker compose build

build-px4:
	DOCKER_BUILDKIT=1 docker compose build px4

build-ros2:
	DOCKER_BUILDKIT=1 docker compose build ros2

build-vio:
	DOCKER_BUILDKIT=1 docker compose build openvins

# ── Simulation Stack ──────────────────────────────────────────
# Phase 1-3: px4 SITL + ros2 + uXRCE-DDS agent
sim-up:
	@echo "==> Starting simulation stack..."
	@xhost +local:docker 2>/dev/null || true
	docker compose --profile sim up -d
	@echo ""
	@echo "  Containers started. Next steps:"
	@echo "    make px4-start     → launch PX4 SITL inside container"
	@echo "    make ros2-shell    → enter ROS2 container"
	@echo "    make health        → verify everything is working"
	@echo ""

# Phase 3+: sim + OpenVINS VIO
vio-up:
	@echo "==> Starting simulation + VIO stack..."
	@xhost +local:docker 2>/dev/null || true
	docker compose --profile sim --profile vio up -d
	@echo "  All containers started (px4, ros2, openvins)"

sim-down:
	@echo "==> Stopping all containers..."
	docker compose --profile sim --profile vio --profile hw down
	@echo "  Done."

# ── Hardware Stack (Phase 5 — Orange Pi) ──────────────────────
hw-up:
	@echo "==> Starting hardware stack (Orange Pi mode)..."
	@echo "    NOTE: Run this on the Orange Pi, not the dev machine."
	docker compose --profile hw up -d

# ── Quick PX4 SITL Launch ─────────────────────────────────────
# Convenience: starts PX4 SITL in the px4 container automatically
# Equivalent to: docker exec px4 bash -c "cd /PX4-Autopilot && make px4_sitl gz_x500"
px4-start:
	@echo "==> Starting PX4 SITL (HEADLESS, gz_x500 model)..."
	docker exec -d px4 bash -c \
		"cd /PX4-Autopilot && \
		 PX4_HEADLESS=$(PX4_HEADLESS) \
		 PX4_SIM_SPEED_FACTOR=$(PX4_SIM_SPEED_FACTOR) \
		 make px4_sitl gz_x500 2>&1 | tee /tmp/px4_sitl.log"
	@echo "  PX4 SITL starting in background..."
	@echo "  Monitor: docker exec px4 tail -f /tmp/px4_sitl.log"
	@echo "  Wait ~30s for 'Ready to fly' message before running ROS2 nodes."

# ── Interactive Shells ────────────────────────────────────────
px4-shell:
	docker exec -it px4 bash

ros2-shell:
	docker exec -it ros2 bash

openvins-shell:
	docker exec -it openvins bash

# ── Monitoring ────────────────────────────────────────────────
health:
	@bash scripts/health_check.sh

logs:
	docker compose logs -f

ps:
	docker compose ps

# ── Git Setup ─────────────────────────────────────────────────
git-init:
	@echo "==> Initializing Git repository..."
	@git init
	@git add .
	@git commit -m "feat: Phase 0 — project scaffold, Docker foundation"
	@git tag v0.1.0
	@echo ""
	@echo "  Git repo initialized with tag v0.1.0"
	@echo "  To push to GitHub:"
	@echo "    git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO.git"
	@echo "    git push -u origin main --tags"
	@echo ""

# ── Cleanup ───────────────────────────────────────────────────
clean:
	docker compose --profile sim --profile vio --profile hw down --rmi local

clean-all:
	docker compose --profile sim --profile vio --profile hw down \
		--rmi all --volumes --remove-orphans
	@echo "  WARNING: All images and volumes removed. Next build will take ~35 min."
