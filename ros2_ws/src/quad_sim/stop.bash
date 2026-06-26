#!/usr/bin/env bash
# =============================================================================
# stop.bash — Stop all Aerostack2 tmux sessions
# =============================================================================
set -euo pipefail

DRONE_NS="${1:-drone0}"
TMUX_SESSION="as2_${DRONE_NS}"

echo "==> Stopping Aerostack2 session: ${TMUX_SESSION}"
tmux kill-session -t "${TMUX_SESSION}" 2>/dev/null && \
    echo "  ✓ Session stopped." || \
    echo "  Session was not running."
