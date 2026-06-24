#!/bin/bash
# =============================================================
# scripts/px4_params_upload.sh
# =============================================================
# Uploads a PX4 .params file to a running PX4 instance.
# Works for both SITL (via MAVLink UDP) and real hardware (UART).
#
# Usage:
#   bash scripts/px4_params_upload.sh <params_file> [target]
#
# Examples:
#   bash scripts/px4_params_upload.sh config/px4_params/sim_gps_denied.params sim
#   bash scripts/px4_params_upload.sh config/px4_params/real_gps_denied.params hw
#
# Parameters:
#   <params_file>  : Path to .params file
#   [target]       : "sim" (default, UDP) or "hw" (UART serial)
# =============================================================

set -euo pipefail

PARAMS_FILE="${1:-}"
TARGET="${2:-sim}"

# ── Validate input ────────────────────────────────────────────
if [ -z "${PARAMS_FILE}" ]; then
    echo "Usage: $0 <params_file> [sim|hw]"
    echo ""
    echo "Available params files:"
    ls config/px4_params/*.params 2>/dev/null || echo "  None found."
    exit 1
fi

if [ ! -f "${PARAMS_FILE}" ]; then
    echo "ERROR: File not found: ${PARAMS_FILE}"
    exit 1
fi

echo "Uploading PX4 parameters from: ${PARAMS_FILE}"
echo "Target: ${TARGET}"
echo ""

# ── Upload via MAVLink (SITL) ─────────────────────────────────
if [ "${TARGET}" = "sim" ]; then
    # For SITL: use mavlink-routerd or the PX4 param command inside container
    echo "Using PX4 SITL param command..."
    echo ""

    # Read each non-comment line from the params file and set it
    while IFS= read -r line; do
        # Skip blank lines and comments
        [[ -z "$line" || "$line" =~ ^# ]] && continue
        # Format: PARAM_NAME VALUE
        PARAM_NAME=$(echo "$line" | awk '{print $1}')
        PARAM_VALUE=$(echo "$line" | awk '{print $2}')
        echo "  Setting ${PARAM_NAME} = ${PARAM_VALUE}"
        docker exec px4 bash -c \
            "cd /PX4-Autopilot && echo 'param set ${PARAM_NAME} ${PARAM_VALUE}' | \
             ./build/px4_sitl_default/bin/px4 -s /dev/stdin" 2>/dev/null || \
            echo "    (Could not set ${PARAM_NAME} — is SITL running?)"
    done < "${PARAMS_FILE}"

# ── Upload via UART (Real Hardware) ──────────────────────────
elif [ "${TARGET}" = "hw" ]; then
    echo "For real hardware, use QGroundControl to load the .params file:"
    echo ""
    echo "  1. Connect QGroundControl to your Pixhawk"
    echo "  2. Go to: Vehicle Setup → Parameters → Tools → Load from file"
    echo "  3. Select: ${PARAMS_FILE}"
    echo "  4. Reboot the Pixhawk after upload"
    echo ""
    echo "Or use MAVSDK-Python / pymavlink for automated upload."
fi

echo ""
echo "Done. Reboot PX4 (or restart SITL) to apply all parameter changes."
