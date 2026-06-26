#!/usr/bin/env bash
# =============================================================================
# launch_as2.bash — Launch Aerostack2 nodes for quad_real (Physical Hardware)
# =============================================================================
# Run INSIDE the hw_stack container on the Orange Pi 5.
# Launches: RealSense driver → OpenVINS → AS2 platform → state estimator
#           → motion controller → behaviours
# =============================================================================

set -euo pipefail

DRONE_NS="${DRONE_NAMESPACE:-drone0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"
CORE_CONFIG="/ros2_ws/src/quad_core/config"
TMUX_SESSION="as2_hw_${DRONE_NS}"

source /opt/ros/humble/setup.bash

echo ""
echo "╔════════════════════════════════════════════════════╗"
echo "║   Aerostack2 HARDWARE — Drone: ${DRONE_NS}           ║"
echo "║   Serial: ${PIXHAWK_SERIAL_PORT:-/dev/ttyUSB0}         ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""

tmux kill-session -t "${TMUX_SESSION}" 2>/dev/null || true
tmux new-session -d -s "${TMUX_SESSION}" -x 220 -y 50

# ── Window 0: RealSense D435i Camera Driver ───────────────────────────────
tmux rename-window -t "${TMUX_SESSION}:0" "realsense"
tmux send-keys -t "${TMUX_SESSION}:0" \
    "source /opt/ros/humble/setup.bash && \
     echo '[realsense] Starting D435i driver...' && \
     ros2 launch realsense2_camera rs_launch.py \
       config_file:=${CONFIG_DIR}/realsense_hw.yaml \
       camera_name:=camera \
       enable_infra1:=true \
       enable_depth:=true \
       enable_color:=false \
       unite_imu_method:=linear_interpolation" Enter

sleep 5   # Wait for camera to initialise

# ── Window 1: MicroXRCE-DDS Agent (serial to Pixhawk) ─────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "dds-agent"
tmux send-keys -t "${TMUX_SESSION}:dds-agent" \
    "echo '[dds-agent] Starting MicroXRCE Agent on serial ${PIXHAWK_SERIAL_PORT:-/dev/ttyUSB0}...' && \
     MicroXRCEAgent serial \
       --dev ${PIXHAWK_SERIAL_PORT:-/dev/ttyUSB0} \
       -b ${PIXHAWK_BAUD_RATE:-921600}" Enter

sleep 3

# ── Window 2: AS2 Platform — Pixhawk Serial Bridge ───────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "platform"
tmux send-keys -t "${TMUX_SESSION}:platform" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_platform_pixhawk pixhawk_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false \
       connection_type:=serial \
       serial_device:=${PIXHAWK_SERIAL_PORT:-/dev/ttyUSB0} \
       serial_baudrate:=${PIXHAWK_BAUD_RATE:-921600} \
       config:=${CONFIG_DIR}/as2_platform_hw.yaml" Enter

sleep 3

# ── Window 3: OpenVINS VIO ────────────────────────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "openvins"
tmux send-keys -t "${TMUX_SESSION}:openvins" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch ov_msckf subscribe.launch.py \
       config_path:=${CORE_CONFIG}/vio_d435i.yaml \
       verbosity:=WARNING \
       use_stereo:=false \
       max_cameras:=1" Enter

sleep 5

# ── Window 4: State Estimator ─────────────────────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "state-est"
tmux send-keys -t "${TMUX_SESSION}:state-est" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_state_estimator state_estimator_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false \
       plugin:=raw_odometry \
       config:=${CONFIG_DIR}/state_estimator_hw.yaml" Enter

sleep 2

# ── Window 5: Motion Controller ──────────────────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "controller"
tmux send-keys -t "${TMUX_SESSION}:controller" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_motion_controller controller_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false \
       plugin:=differential_flatness_controller" Enter

sleep 2

# ── Window 6: Behaviour Servers ──────────────────────────────────────────
tmux new-window -t "${TMUX_SESSION}" -n "behaviours"
tmux send-keys -t "${TMUX_SESSION}:behaviours" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
       namespace:=${DRONE_NS} \
       use_sim_time:=false" Enter

echo ""
echo "  ✓ Hardware stack launching in tmux session: ${TMUX_SESSION}"
echo ""
echo "  Attach with: tmux attach -t ${TMUX_SESSION}"
echo ""
echo "  Pre-flight checklist:"
echo "    □ Pixhawk connected and powered"
echo "    □ RealSense D435i detected (window 0 shows camera topics)"
echo "    □ OpenVINS initialised (window 3 shows odometry)"
echo "    □ EKF2 health OK in QGroundControl"
echo "    □ All props removed for first test"
echo "  Then: make mission"
echo ""

tmux attach-session -t "${TMUX_SESSION}"
