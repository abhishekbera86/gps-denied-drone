#!/usr/bin/env bash
# =============================================================================
# health_check.sh — Runtime Health Verification
# =============================================================================
# Checks that all required containers are running and that the critical
# ROS 2 and PX4 topics are live.
#
# Usage: make health  (or run directly: bash scripts/health_check.sh)
# =============================================================================

set -euo pipefail

PASS=0
FAIL=0
WARN=0

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No colour

pass() { echo -e "  ${GREEN}✓${NC} $1"; PASS=$((PASS+1)); }
fail() { echo -e "  ${RED}✗${NC} $1"; FAIL=$((FAIL+1)); }
warn() { echo -e "  ${YELLOW}⚠${NC} $1"; WARN=$((WARN+1)); }

echo ""
echo "╔════════════════════════════════════════════════════╗"
echo "║         GPS-Denied Drone Stack — Health Check       ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""

# ── 1. Docker Containers ─────────────────────────────────────────────────
echo "[ Containers ]"

for svc in px4_sitl aerostack2; do
    if docker inspect --format='{{.State.Status}}' "${svc}" 2>/dev/null | grep -q "running"; then
        pass "${svc} is running"
    else
        fail "${svc} is NOT running — run: make sim-up"
    fi
done

# Optional VIO container
if docker inspect --format='{{.State.Status}}' openvins 2>/dev/null | grep -q "running"; then
    pass "openvins is running"
else
    warn "openvins not running (expected if --profile vio not started)"
fi

echo ""

# ── 2. MicroXRCE-DDS Agent ────────────────────────────────────────────────
echo "[ MicroXRCE-DDS Agent ]"

if docker exec aerostack2 pgrep -x MicroXRCEAgent > /dev/null 2>&1; then
    pass "MicroXRCEAgent process is running"
else
    fail "MicroXRCEAgent NOT running in aerostack2 container"
fi

echo ""

# ── 3. PX4 SITL Process ───────────────────────────────────────────────────
echo "[ PX4 SITL ]"

if docker exec px4_sitl pgrep -x px4 > /dev/null 2>&1; then
    pass "PX4 process is running"
else
    warn "PX4 process not yet running — it may still be starting (~60s)"
    warn "Monitor with: docker logs -f px4_sitl | grep 'Ready to fly'"
fi

echo ""

# ── 4. ROS 2 Topics ──────────────────────────────────────────────────────
echo "[ ROS 2 Topics (DDS bridge) ]"

check_topic() {
    local topic="$1"
    local label="$2"
    if docker exec aerostack2 bash -c \
        "source /opt/ros/humble/setup.bash && \
         ros2 topic info ${topic} --no-daemon 2>/dev/null | grep -q 'Publisher'" 2>/dev/null; then
        pass "${label}: ${topic}"
    else
        fail "${label}: ${topic} (no publisher — DDS bridge may not be connected)"
    fi
}

# PX4 → ROS 2 topics (require DDS bridge to be working)
check_topic "/fmu/out/vehicle_odometry"       "PX4 odometry"
check_topic "/fmu/out/vehicle_status"         "PX4 vehicle status"
check_topic "/fmu/out/vehicle_local_position" "PX4 local position"

echo ""

# ── 5. Aerostack2 Nodes ───────────────────────────────────────────────────
echo "[ Aerostack2 Nodes ]"

check_topic "/drone0/self_localization/pose"  "AS2 state estimator"

echo ""

# ── 6. VIO (if openvins running) ─────────────────────────────────────────
if docker inspect --format='{{.State.Status}}' openvins 2>/dev/null | grep -q "running"; then
    echo "[ OpenVINS VIO ]"
    check_topic "/openvins/odometry"  "VIO odometry"
    echo ""
fi

# ── Summary ───────────────────────────────────────────────────────────────
echo "════════════════════════════════════════════════════════"
echo -e "  ${GREEN}PASS: ${PASS}${NC}   ${RED}FAIL: ${FAIL}${NC}   ${YELLOW}WARN: ${WARN}${NC}"
echo "════════════════════════════════════════════════════════"
echo ""

if [[ ${FAIL} -gt 0 ]]; then
    echo "  Some checks failed. Common fixes:"
    echo "    make sim-up          → start containers if not running"
    echo "    docker logs px4_sitl → check PX4 startup errors"
    echo "    docker logs aerostack2 → check DDS agent startup"
    echo ""
    exit 1
else
    echo "  All critical checks passed. ✓"
    echo "  Ready to run: make as2-launch, then make mission"
    echo ""
fi
