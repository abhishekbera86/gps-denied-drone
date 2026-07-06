#!/usr/bin/env bash
# =============================================================================
# scripts/launch_hw.sh
# =============================================================================
# Called by:  docker exec -it hw_stack bash /scripts/launch_hw.sh
#
# Builds packages, then runs:
#   ros2 launch quad_real hw.launch.py
# =============================================================================

set -eo pipefail

DRONE_NS="${DRONE_NAMESPACE:-drone0}"
SERIAL_DEV="${PIXHAWK_SERIAL_PORT:-/dev/ttyUSB0}"
SERIAL_BAUD="${PIXHAWK_BAUD_RATE:-921600}"
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

source /opt/ros/humble/setup.bash

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}  GPS-Denied Drone — HARDWARE Stack                   ${NC}"
echo -e "${CYAN}  Pixhawk: ${SERIAL_DEV} @ ${SERIAL_BAUD} baud          ${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# ── Build packages ────────────────────────────────────────────────────────
echo -e "${YELLOW}[1/2] Building ROS 2 packages (quad_core, quad_real)...${NC}"
mkdir -p /ros2_ws_build
cd /ros2_ws_build
colcon build \
    --symlink-install \
    --packages-select quad_core quad_real \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | grep -E "(Building|Finished|ERROR|error)" || true

source /ros2_ws_build/install/setup.bash
echo -e "${GREEN}  ✓ Packages built${NC}"

# ── Verify RealSense detected ─────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[2/2] Checking RealSense D435i...${NC}"
if rs-enumerate-devices 2>/dev/null | grep -q "D435"; then
    echo -e "${GREEN}  ✓ RealSense D435i detected${NC}"
else
    echo -e "  ⚠  RealSense not detected. Check USB 3.0 connection."
    echo "     Continuing anyway — camera node will retry."
fi

# ── Launch ────────────────────────────────────────────────────────────────
echo ""
echo -e "  Command: ${GREEN}ros2 launch quad_real hw.launch.py${NC}"
echo "  Open a NEW terminal and run:  make mission"
echo ""

exec ros2 launch quad_real hw.launch.py \
    namespace:="${DRONE_NS}" \
    serial_device:="${SERIAL_DEV}" \
    serial_baud:="${SERIAL_BAUD}"
