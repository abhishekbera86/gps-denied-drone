#!/usr/bin/env bash
# =============================================================================
# entrypoint_ros2_autonomy.sh
# =============================================================================
# Starts the Micro-XRCE-DDS-Agent in the background so the PX4 <-> ROS 2
# bridge is live as soon as the container is up — no manual `docker exec`
# step required before running any ROS 2 node against /fmu/* topics.
# =============================================================================
set -e

# Trim whitespace: a padded value (e.g. from an inline comment in .env)
# makes MicroXRCEAgent reject the port and die silently at container boot.
UXRCE_DDS_PORT="$(echo "${UXRCE_DDS_PORT:-8888}" | tr -d '[:space:]')"

MicroXRCEAgent udp4 -p "${UXRCE_DDS_PORT}" > /var/log/microxrce_agent.log 2>&1 &
echo "Micro-XRCE-DDS-Agent started on udp4 port ${UXRCE_DDS_PORT} (log: /var/log/microxrce_agent.log)"

exec "$@"
