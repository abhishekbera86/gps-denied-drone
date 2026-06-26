#!/usr/bin/env bash
# =============================================================================
# launch_as2.bash — Launch Aerostack2 nodes for quad_sim (GPS-denied, VIO)
# =============================================================================
# This script launches all Aerostack2 ROS 2 nodes for a single drone
# in a tmux session. It is designed to run INSIDE the aerostack2 container.
#
# Usage:
#   ./launch_as2.bash                   # Launch for drone0
#   ./launch_as2.bash -n drone0         # Explicit namespace
#   ./launch_as2.bash -s                # Skip simulation launch (nodes only)
#
# Options:
#   -n <namespace>   Drone namespace (default: drone0)
#   -s               Skip Gazebo/SITL launch (assume already running)
#   -h               Show this help message
# =============================================================================

set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────
DRONE_NS="drone0"
SKIP_SIM="false"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"
TMUX_SESSION="as2_${DRONE_NS}"

usage() {
    echo "Usage: $0 [-n <namespace>] [-s] [-h]"
    echo "  -n <namespace>  Drone namespace (default: drone0)"
    echo "  -s              Skip simulation launch"
    echo "  -h              Show this help"
}

while getopts "n:sh" opt; do
    case ${opt} in
        n) DRONE_NS="${OPTARG}" ;;
        s) SKIP_SIM="true" ;;
        h) usage; exit 0 ;;
        *) usage; exit 1 ;;
    esac
done

# ── Source ROS 2 ──────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
# Source Aerostack2 workspace (pre-installed in image)
source /aerostack2_ws/install/setup.bash 2>/dev/null || true

echo ""
echo "╔════════════════════════════════════════════════════╗"
echo "║   Launching Aerostack2 — Drone: ${DRONE_NS}          ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""

# ── Kill any existing tmux session for this drone ─────────────────────────
tmux kill-session -t "${TMUX_SESSION}" 2>/dev/null || true

# ── Create tmux session ───────────────────────────────────────────────────
tmux new-session -d -s "${TMUX_SESSION}" -x 220 -y 50

# ── Window 0: MicroXRCE-DDS Agent status ──────────────────────────────────
# (Agent was started by docker-compose command — this window just monitors it)
tmux rename-window -t "${TMUX_SESSION}:0" "dds-agent"
tmux send-keys -t "${TMUX_SESSION}:0" \
    "echo '[dds-agent] Monitoring MicroXRCE-DDS Agent...' && \
     watch -n2 'pgrep -x MicroXRCEAgent && echo RUNNING || echo NOT RUNNING'" Enter

# ── Window 1: AS2 Platform — Pixhawk Bridge ───────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "platform"
tmux send-keys -t "${TMUX_SESSION}:platform" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_platform_pixhawk pixhawk_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false \
       connection_type:=udp \
       udp_ip:=127.0.0.1 \
       udp_port:=8888 \
       config:=${CONFIG_DIR}/as2_platform_sim.yaml" Enter

# Wait for platform to start
sleep 3

# ── Window 2: State Estimator (VIO ingestion) ─────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "state-est"
tmux send-keys -t "${TMUX_SESSION}:state-est" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_state_estimator state_estimator_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false \
       plugin:=raw_odometry \
       config:=${CONFIG_DIR}/state_estimator_sim.yaml" Enter

sleep 2

# ── Window 3: Motion Controller ───────────────────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "controller"
tmux send-keys -t "${TMUX_SESSION}:controller" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_motion_controller controller_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false \
       plugin:=differential_flatness_controller \
       config:=${CONFIG_DIR}/motion_controller_sim.yaml" Enter

sleep 2

# ── Window 4: Behaviour Servers ───────────────────────────────────────────
# All basic motion behaviours as ROS 2 action servers.
tmux new-window -t "${TMUX_SESSION}" -n "behaviours"
tmux send-keys -t "${TMUX_SESSION}:behaviours" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false" Enter

sleep 2

# ── Window 5: ROS 2 topic monitor ────────────────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "monitor"
tmux send-keys -t "${TMUX_SESSION}:monitor" \
    "source /opt/ros/humble/setup.bash && \
     watch -n1 'ros2 topic echo /${DRONE_NS}/self_localization/pose --once 2>/dev/null'" Enter

echo ""
echo "  ✓ Aerostack2 nodes launching in tmux session: ${TMUX_SESSION}"
echo ""
echo "  To attach to the tmux session:"
echo "    tmux attach -t ${TMUX_SESSION}"
echo ""
echo "  Windows:"
echo "    0: dds-agent   — MicroXRCE-DDS Agent monitor"
echo "    1: platform    — as2_platform_pixhawk (PX4 bridge)"
echo "    2: state-est   — State estimator (VIO → AS2 state)"
echo "    3: controller  — Motion controller (differential flatness)"
echo "    4: behaviours  — Motion behaviours (Takeoff, Land, GoTo)"
echo "    5: monitor     — Live self_localization topic echo"
echo ""
echo "  Wait ~10s for all nodes to initialize, then run:"
echo "    make mission"
echo ""

# ── Attach to session ─────────────────────────────────────────────────────
tmux attach-session -t "${TMUX_SESSION}"
