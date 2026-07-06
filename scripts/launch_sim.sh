#!/usr/bin/env bash
# =============================================================================
# scripts/launch_sim.sh
# =============================================================================
# Called by:  docker exec -it aerostack2 bash /scripts/launch_sim.sh
#
# Steps:
#   1. Source all workspaces (AS2 base + built quad packages)
#   2. Build quad_core + quad_sim via colcon (fast: Python-only, ~10s)
#   3. Run:  ros2 launch quad_sim sim.launch.py
#      → stays in foreground so you see all node logs
#
# In a second terminal, run:  make mission
# =============================================================================

set -euo pipefail

DRONE_NS="${DRONE_NAMESPACE:-drone0}"
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

# ── Source workspaces ─────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash 2>/dev/null || true
source /px4_platform_ws/install/setup.bash 2>/dev/null || true

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}  GPS-Denied Drone — Simulation World                 ${NC}"
echo -e "${CYAN}  Platform: as2_platform_multirotor_simulator          ${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# ── Build quad_core + quad_sim so ros2 launch finds them by package name ──
echo -e "${YELLOW}[1/2] Building ROS 2 packages (quad_core, quad_sim)...${NC}"
echo "      (Python-only — takes ~10 seconds, cached after first run)"

# Build into /ros2_ws_build (persisted via Docker volume — colcon_build_cache)
mkdir -p /ros2_ws_build
cd /ros2_ws_build

# Source path: /ros2_ws/src (read-only mount from host)
colcon build \
    --symlink-install \
    --base-paths /ros2_ws/src \
    --build-base /ros2_ws_build/build \
    --install-base /ros2_ws_build/install \
    --packages-select quad_core quad_sim \
    2>&1 | grep -E "(Building|Finished|ERROR|error|WARNING)" || true

source /ros2_ws_build/install/setup.bash
echo -e "${GREEN}  ✓ Packages ready${NC}"

# ── Launch the simulation world ───────────────────────────────────────────
echo ""
echo -e "${YELLOW}[2/2] Launching simulation world...${NC}"
echo -e "  Command: ${GREEN}ros2 launch quad_sim sim.launch.py namespace:=${DRONE_NS}${NC}"
echo ""
echo "  This terminal shows all node logs."
echo -e "  Open a ${GREEN}NEW terminal${NC} and run:  ${GREEN}make mission${NC}"
echo ""

# Runs in foreground — Ctrl+C stops all nodes cleanly.
exec ros2 launch quad_sim sim.launch.py namespace:="${DRONE_NS}"
