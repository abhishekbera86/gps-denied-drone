#!/usr/bin/env bash
# =============================================================================
# scripts/launch_sim.sh
# =============================================================================
# Called by:  docker exec -it aerostack2 bash /scripts/launch_sim.sh
#
# What it does:
#   1. Builds the ROS 2 packages (quad_core, quad_sim) via colcon
#      so ros2 launch can find them by package name.
#      (build output is in /ros2_ws/build — not mounted, stays in container)
#   2. Waits for PX4 SITL to publish DDS topics.
#   3. Runs:  ros2 launch quad_sim sim.launch.py
#      This is the proper ROS 2 launch file — it stays in the foreground
#      so you can see all node output in this terminal.
#
# Usage:
#   Terminal 1:  make sim        (stays in foreground — shows all ROS 2 logs)
#   Terminal 2:  make mission    (runs the mission script)
# =============================================================================

set -euo pipefail

DRONE_NS="${DRONE_NAMESPACE:-drone0}"
PX4_READY_TIMEOUT=120
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

# ── Source ROS 2 ──────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source /aerostack2_ws/install/setup.bash 2>/dev/null || true

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}  GPS-Denied Drone — Simulation World                 ${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# ── Step 1: Build packages so ros2 launch can find them ──────────────────
# Packages are in /ros2_ws/src (mounted from host, read-only).
# We copy-build into /ros2_ws_build (inside container, writable).
echo -e "${YELLOW}[1/3] Building ROS 2 packages (quad_core, quad_sim)...${NC}"
echo "      (fast: only Python packages — no C++ compilation)"

mkdir -p /ros2_ws_build

cd /ros2_ws_build
colcon build \
    --symlink-install \
    --packages-select quad_core quad_sim \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | grep -E "(Building|Finished|ERROR|error)" || true

source /ros2_ws_build/install/setup.bash
echo -e "${GREEN}  ✓ Packages built and sourced${NC}"

# ── Step 2: Wait for PX4 SITL to boot ────────────────────────────────────
echo ""
echo -e "${YELLOW}[2/3] Waiting for PX4 SITL DDS topics...${NC}"
echo "      (PX4 publishes topics once it reaches 'Ready to fly')"

elapsed=0
while true; do
    topic_count=$(ros2 topic list 2>/dev/null | grep -c "fmu/out" || true)
    if [[ "${topic_count}" -gt 0 ]]; then
        echo -e "${GREEN}  ✓ PX4 is online (${topic_count} fmu/out topics visible)${NC}"
        break
    fi
    if [[ ${elapsed} -ge ${PX4_READY_TIMEOUT} ]]; then
        echo "  ERROR: PX4 did not come up after ${PX4_READY_TIMEOUT}s."
        echo "  Check:  docker logs px4_sitl"
        exit 1
    fi
    printf "  Waiting... (%ds / %ds)\r" "${elapsed}" "${PX4_READY_TIMEOUT}"
    sleep 3
    elapsed=$((elapsed + 3))
done

# ── Step 3: Launch the simulation world ──────────────────────────────────
echo ""
echo -e "${YELLOW}[3/3] Launching simulation world...${NC}"
echo -e "      Command: ${GREEN}ros2 launch quad_sim sim.launch.py namespace:=${DRONE_NS}${NC}"
echo ""
echo "  This terminal shows all node logs."
echo "  Open a NEW terminal and run:  make mission"
echo ""

# This runs in the FOREGROUND — Ctrl+C stops all nodes cleanly.
exec ros2 launch quad_sim sim.launch.py \
    namespace:="${DRONE_NS}"
