# =============================================================================
# Makefile — GPS-Denied Autonomous Drone Stack
# =============================================================================
# Convenience wrapper around docker compose and docker exec commands.
# Always run from the project root (px4_docker_ws/).
#
# Usage:
#   make help        → show all commands
#   make build       → build all images
#   make sim-up      → start simulation
#   make as2-launch  → start Aerostack2 nodes inside aerostack2 container
#   make mission     → run the autonomous mission
# =============================================================================

include .env
export

.PHONY: help \
        build build-px4 build-as2 build-vio build-hw \
        sim-up sim-down vio-up hw-up \
        as2-launch as2-stop \
        mission params-upload \
        shell-px4 shell-as2 shell-openvins shell-hw \
        health logs ps \
        clean clean-all

# ── Default target ────────────────────────────────────────────────────────
help:
	@echo ""
	@echo "  ╔════════════════════════════════════════════════════╗"
	@echo "  ║   GPS-Denied Autonomous Drone — Make Targets       ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  BUILD                                             ║"
	@echo "  ║    build          Build all Docker images          ║"
	@echo "  ║    build-px4      Build only px4_sitl image        ║"
	@echo "  ║    build-as2      Build only aerostack2 image      ║"
	@echo "  ║    build-vio      Build only openvins image        ║"
	@echo "  ║    build-hw       Build hw-stack image (ARM64)     ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  SIMULATION (no GPU / headless)                    ║"
	@echo "  ║    sim-up         Start PX4 SITL + Aerostack2      ║"
	@echo "  ║    vio-up         Start sim + OpenVINS VIO         ║"
	@echo "  ║    sim-down       Stop all containers              ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  AEROSTACK2 NODES                                  ║"
	@echo "  ║    as2-launch     Start AS2 nodes in container     ║"
	@echo "  ║    as2-stop       Stop AS2 nodes (tmux sessions)   ║"
	@echo "  ║    mission        Run mission.py from quad_core    ║"
	@echo "  ║    params-upload  Upload EKF2 VIO params to PX4   ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  SHELLS (interactive bash)                         ║"
	@echo "  ║    shell-px4      Open bash in px4_sitl            ║"
	@echo "  ║    shell-as2      Open bash in aerostack2          ║"
	@echo "  ║    shell-openvins Open bash in openvins            ║"
	@echo "  ║    shell-hw       Open bash in hw_stack            ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  MONITORING                                        ║"
	@echo "  ║    health         Run health check                 ║"
	@echo "  ║    logs           Tail logs from all containers    ║"
	@echo "  ║    ps             Show container status            ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  HARDWARE (Orange Pi 5 — run ON the companion PC)  ║"
	@echo "  ║    hw-up          Start hardware stack             ║"
	@echo "  ╠════════════════════════════════════════════════════╣"
	@echo "  ║  MAINTENANCE                                       ║"
	@echo "  ║    clean          Stop + remove local images       ║"
	@echo "  ║    clean-all      Stop + remove images + volumes   ║"
	@echo "  ╚════════════════════════════════════════════════════╝"
	@echo ""

# ── Build ─────────────────────────────────────────────────────────────────
build:
	@echo "==> Building all Docker images (first build: ~5-10 min)..."
	@echo "    px4_sitl: px4io base + PX4 v${PX4_VERSION} SITL baked in"
	@echo "    aerostack2: nightly-humble base + apt AS2 packages"
	@echo "    openvins: ros:humble base + apt OpenVINS"
	DOCKER_BUILDKIT=1 docker compose build

build-px4:
	@echo "==> Building px4_sitl image..."
	DOCKER_BUILDKIT=1 docker compose build px4_sitl

build-as2:
	@echo "==> Building aerostack2 image..."
	DOCKER_BUILDKIT=1 docker compose build aerostack2

build-vio:
	@echo "==> Building openvins image..."
	DOCKER_BUILDKIT=1 docker compose build openvins

build-hw:
	@echo "==> Building hw_stack image (ARM64)..."
	@echo "    NOTE: Cross-compile with: docker buildx build --platform linux/arm64"
	@echo "    Or run this on the Orange Pi 5 directly."
	DOCKER_BUILDKIT=1 docker compose build hw_stack

# ── Simulation ────────────────────────────────────────────────────────────
sim-up:
	@echo "==> Starting simulation stack (headless, no GPU required)..."
	@echo "    PX4 SITL + Gazebo Garden + Aerostack2 + DDS Agent"
	docker compose --profile sim up -d
	@echo ""
	@echo "  Containers starting. Next steps:"
	@echo "    make health      → verify all containers are healthy"
	@echo "    make as2-launch  → start Aerostack2 nodes"
	@echo "    make mission     → run mission.py"
	@echo ""
	@echo "  Monitor PX4 SITL startup:"
	@echo "    docker logs -f px4_sitl"
	@echo "  Watch for: 'Ready to fly' before running missions."
	@echo ""

vio-up:
	@echo "==> Starting simulation + VIO stack..."
	docker compose --profile sim --profile vio up -d
	@echo "  All containers started: px4_sitl, aerostack2, openvins"

sim-down:
	@echo "==> Stopping all containers..."
	docker compose --profile sim --profile vio --profile hw down
	@echo "  Done."

# ── Aerostack2 Node Management ────────────────────────────────────────────
# Launches Aerostack2 nodes inside the running aerostack2 container.
# Uses the quad_sim world config (single drone, GPS-denied VIO).
as2-launch:
	@echo "==> Launching Aerostack2 nodes (drone0) inside container..."
	docker exec -it aerostack2 bash -c "\
		source /opt/ros/humble/setup.bash && \
		cd /ros2_ws/src/quad_sim && \
		./launch_as2.bash -n drone0 \
	"

as2-stop:
	@echo "==> Stopping Aerostack2 tmux sessions..."
	docker exec -it aerostack2 bash -c "\
		cd /ros2_ws/src/quad_sim && ./stop.bash \
	"

# ── Mission Execution ─────────────────────────────────────────────────────
mission:
	@echo "==> Running autonomous mission from quad_core..."
	docker exec -it aerostack2 bash -c "\
		source /opt/ros/humble/setup.bash && \
		python3 /ros2_ws/src/quad_core/mission.py \
	"

# ── Parameter Upload ──────────────────────────────────────────────────────
# Uploads EKF2 GPS-denied VIO parameters to the running PX4 instance.
# Run AFTER PX4 SITL shows 'Ready to fly', or after connecting to real drone.
params-upload:
	@echo "==> Uploading EKF2 VIO parameters to PX4..."
	docker exec -it aerostack2 python3 /scripts/upload_px4_params.py \
		--params /ros2_ws/src/quad_core/config/ekf2_vio.params \
		--port ${MAVLINK_GCS_PORT}

# ── Interactive Shells ────────────────────────────────────────────────────
shell-px4:
	docker exec -it px4_sitl bash

shell-as2:
	docker exec -it aerostack2 bash

shell-openvins:
	docker exec -it openvins bash

shell-hw:
	docker exec -it hw_stack bash

# ── Hardware Stack (Orange Pi 5) ──────────────────────────────────────────
hw-up:
	@echo "==> Starting hardware stack..."
	@echo "    NOTE: Run this ON the Orange Pi 5, not the dev machine."
	docker compose --profile hw up -d
	@echo "  Container started. Next steps:"
	@echo "    make shell-hw    → enter hw_stack container"
	@echo "    make as2-launch  → start AS2 nodes (hw config)"
	@echo "    make mission     → run identical mission.py"

# ── Monitoring ────────────────────────────────────────────────────────────
health:
	@bash scripts/health_check.sh

logs:
	docker compose logs -f

ps:
	docker compose ps

# ── Maintenance ───────────────────────────────────────────────────────────
clean:
	docker compose --profile sim --profile vio --profile hw down --rmi local

clean-all:
	@echo "WARNING: This removes all images and the ccache volume."
	@echo "Next build will re-download images and recompile PX4 SITL (~5 min)."
	docker compose --profile sim --profile vio --profile hw down \
		--rmi all --volumes --remove-orphans
