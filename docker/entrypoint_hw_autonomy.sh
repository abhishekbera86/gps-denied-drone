#!/usr/bin/env bash
# =============================================================================
# entrypoint_hw_autonomy.sh
# =============================================================================
# Starts the Micro-XRCE-DDS-Agent (serial, over the Pixhawk USB link) in the
# background so the PX4 <-> ROS 2 bridge is live as soon as the container is
# up — same reasoning, same pattern as entrypoint_ros2_autonomy.sh (sim's
# UDP equivalent), fixing a real design mistake: an earlier version of this
# project tied the agent's lifecycle to hw.launch.py's own mission launch
# instead, on the theory that the agent needs .env's PIXHAWK_SERIAL_PORT to
# already be correct anyway. That's true, but it also meant the one moment
# hardware bring-up needs verified-live topics the most -- BEFORE ever
# running a mission, during dry-run bench testing -- was exactly the moment
# they weren't available without starting an agent by hand. Confirmed live
# during actual hardware bring-up as a real, avoidable source of confusion.
#
# Unlike sim's UDP agent (always "available" -- no physical device to wait
# for), a real serial device may not exist yet when this container starts
# (Pixhawk not plugged in, or docker-compose's `devices:` mapping pointing
# at a stale path from before .env was corrected -- see resource/
# hardware-bringup-gps.md's Sec9.2 for that exact gotcha). Warn and continue
# rather than fail the whole container in that case -- the device can be
# fixed and the agent started by hand without needing to rebuild anything.
# =============================================================================
set -e

PIXHAWK_SERIAL_PORT="$(echo "${PIXHAWK_SERIAL_PORT:-/dev/ttyUSB0}" | tr -d '[:space:]')"
PIXHAWK_BAUD_RATE="$(echo "${PIXHAWK_BAUD_RATE:-921600}" | tr -d '[:space:]')"

if [ -e "${PIXHAWK_SERIAL_PORT}" ]; then
    MicroXRCEAgent serial --dev "${PIXHAWK_SERIAL_PORT}" -b "${PIXHAWK_BAUD_RATE}" \
        > /var/log/microxrce_agent.log 2>&1 &
    echo "Micro-XRCE-DDS-Agent started on serial ${PIXHAWK_SERIAL_PORT} @ ${PIXHAWK_BAUD_RATE} (log: /var/log/microxrce_agent.log)"
else
    echo "WARNING: ${PIXHAWK_SERIAL_PORT} not found at container start -- Micro-XRCE-DDS-Agent NOT started."
    echo "  Most likely cause: the Pixhawk wasn't plugged in yet, or .env was edited AFTER this"
    echo "  container was created (docker-compose's device mapping is fixed at creation time --"
    echo "  see resource/hardware-bringup-gps.md Sec9.2). Fix the device/.env, then either:"
    echo "    docker compose --profile hw up -d      # recreate the container so the mapping updates"
    echo "  or, without recreating, start it by hand once the device is confirmed present:"
    echo "    MicroXRCEAgent serial --dev <device> -b ${PIXHAWK_BAUD_RATE}"
fi

exec "$@"
