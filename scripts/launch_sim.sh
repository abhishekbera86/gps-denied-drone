#!/usr/bin/env bash
# =============================================================================
# scripts/launch_sim.sh
# =============================================================================
# ONE-SHOT simulation launcher. Runs inside the aerostack2 container.
# Called by:  docker exec aerostack2 bash /scripts/launch_sim.sh
#
# What it does, in order:
#   1. Waits for PX4 SITL to be fully booted ("Ready to fly")
#   2. Starts all 4 Aerostack2 nodes in a background tmux session
#   3. Prints a "ready" banner so the user knows they can run the mission
# =============================================================================

set -euo pipefail

DRONE_NS="${DRONE_NAMESPACE:-drone0}"
CONFIG_DIR="/ros2_ws/src/quad_sim/config"
TMUX_SESSION="sim"
PX4_READY_TIMEOUT=120   # seconds to wait for PX4 boot

# ── Colours ──────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

# ── Source ROS 2 ──────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source /aerostack2_ws/install/setup.bash 2>/dev/null || true

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}  GPS-Denied Drone — Simulation World Starting         ${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# ── Step 1: Wait for PX4 to boot ─────────────────────────────────────────
echo -e "${YELLOW}[1/3] Waiting for PX4 SITL to be ready...${NC}"
echo "      (timeout: ${PX4_READY_TIMEOUT}s  — PX4 publishes DDS topics when ready)"

elapsed=0
while true; do
    # Check if PX4 is publishing vehicle_status over DDS
    topic_count=$(ros2 topic list 2>/dev/null | grep -c "fmu/out" || true)
    if [[ "${topic_count}" -gt 0 ]]; then
        echo -e "${GREEN}  ✓ PX4 is online — DDS topics visible${NC}"
        break
    fi
    if [[ ${elapsed} -ge ${PX4_READY_TIMEOUT} ]]; then
        echo "  ERROR: PX4 did not come up within ${PX4_READY_TIMEOUT}s."
        echo "  Check logs with:  docker logs px4_sitl"
        exit 1
    fi
    printf "  Waiting... (%ds)\r" "${elapsed}"
    sleep 2
    elapsed=$((elapsed + 2))
done

# ── Step 2: Kill any stale tmux session, start fresh ─────────────────────
echo ""
echo -e "${YELLOW}[2/3] Starting Aerostack2 nodes...${NC}"
tmux kill-session -t "${TMUX_SESSION}" 2>/dev/null || true
tmux new-session -d -s "${TMUX_SESSION}" -x 220 -y 50

# Window 0: AS2 Platform (PX4 bridge via UDP)
tmux rename-window -t "${TMUX_SESSION}:0" "platform"
tmux send-keys -t "${TMUX_SESSION}:0" "
source /opt/ros/humble/setup.bash
ros2 launch as2_platform_pixhawk pixhawk_launch.py \
  namespace:=${DRONE_NS} use_sim_time:=false \
  connection_type:=udp udp_ip:=127.0.0.1 udp_port:=8888 \
  config:=${CONFIG_DIR}/as2_platform_sim.yaml
" Enter
sleep 3

# Window 1: State Estimator (VIO ingestion via raw_odometry plugin)
tmux new-window -t "${TMUX_SESSION}" -n "state-est"
tmux send-keys -t "${TMUX_SESSION}:state-est" "
source /opt/ros/humble/setup.bash
ros2 launch as2_state_estimator state_estimator_launch.py \
  namespace:=${DRONE_NS} use_sim_time:=false \
  plugin:=raw_odometry \
  config:=${CONFIG_DIR}/state_estimator_sim.yaml
" Enter
sleep 2

# Window 2: Motion Controller
tmux new-window -t "${TMUX_SESSION}" -n "controller"
tmux send-keys -t "${TMUX_SESSION}:controller" "
source /opt/ros/humble/setup.bash
ros2 launch as2_motion_controller controller_launch.py \
  namespace:=${DRONE_NS} use_sim_time:=false \
  plugin:=differential_flatness_controller \
  config:=${CONFIG_DIR}/motion_controller_sim.yaml
" Enter
sleep 2

# Window 3: Behaviour Action Servers (Takeoff, Land, GoTo, FollowPath)
tmux new-window -t "${TMUX_SESSION}" -n "behaviours"
tmux send-keys -t "${TMUX_SESSION}:behaviours" "
source /opt/ros/humble/setup.bash
ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
  namespace:=${DRONE_NS} use_sim_time:=false
" Enter
sleep 3

# ── Step 3: Confirm nodes are alive ──────────────────────────────────────
echo -e "${YELLOW}[3/3] Confirming nodes are up...${NC}"
sleep 2

node_count=$(ros2 node list 2>/dev/null | grep -c "${DRONE_NS}" || true)
echo -e "${GREEN}  ✓ ${node_count} Aerostack2 nodes running for namespace /${DRONE_NS}${NC}"

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  ✓ Simulation world is READY                          ${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  Run the mission in a new terminal:"
echo -e "    ${GREEN}make mission${NC}"
echo ""
echo "  Debug nodes (tmux session inside container):"
echo -e "    ${GREEN}make shell-as2${NC}   then:  tmux attach -t ${TMUX_SESSION}"
echo "    Windows: 0=platform  1=state-est  2=controller  3=behaviours"
echo ""
