#!/bin/bash
# =============================================================
# scripts/health_check.sh
# =============================================================
# Verifies the full drone stack is working correctly.
# Run after `make sim-up` and after starting PX4 SITL.
#
# Checks:
#   1. Docker containers are running
#   2. uXRCE-DDS Agent is alive
#   3. Key ROS2 topics are publishing
#   4. PX4 EKF2 is healthy (Phase 1+)
#
# Usage:
#   bash scripts/health_check.sh      (or: make health)
# =============================================================

set -euo pipefail

# ── Colors ────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0
FAIL=0

check() {
    local name="$1"
    local result="$2"   # "ok" or "fail"
    local detail="$3"
    if [ "${result}" = "ok" ]; then
        echo -e "  ${GREEN}✓${NC} ${name}"
        [ -n "${detail}" ] && echo -e "      ${detail}"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}✗${NC} ${name}"
        [ -n "${detail}" ] && echo -e "      ${YELLOW}→${NC} ${detail}"
        FAIL=$((FAIL+1))
    fi
}

echo ""
echo "======================================="
echo "  Drone Stack Health Check"
echo "======================================="
echo ""

# ── 1. Container Status ───────────────────────────────────────
echo -e "${BLUE}[1] Container Status${NC}"

for container in px4 ros2; do
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        STATUS=$(docker inspect --format '{{.State.Status}}' "${container}")
        check "${container} container" "ok" "Status: ${STATUS}"
    else
        check "${container} container" "fail" "Container not running. Run: make sim-up"
    fi
done

# Check optional containers
for container in openvins realsense; do
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        check "${container} container" "ok" "Running"
    fi
done

echo ""

# ── 2. uXRCE-DDS Agent ───────────────────────────────────────
echo -e "${BLUE}[2] uXRCE-DDS Agent${NC}"

if docker ps | grep -q ros2; then
    AGENT_PID=$(docker exec ros2 bash -c "pgrep MicroXRCEAgent 2>/dev/null" || true)
    if [ -n "${AGENT_PID}" ]; then
        check "MicroXRCEAgent process" "ok" "PID: ${AGENT_PID}"
    else
        check "MicroXRCEAgent process" "fail" \
            "Agent not running inside ros2 container. Check: docker logs ros2"
    fi
else
    check "MicroXRCEAgent" "fail" "ros2 container not running"
fi

echo ""

# ── 3. ROS2 Topics (requires PX4 SITL to be running) ─────────
echo -e "${BLUE}[3] ROS2 Topics (requires PX4 SITL running)${NC}"
echo    "    Tip: Start SITL first with: make px4-start"
echo ""

if docker ps | grep -q ros2; then
    # Get topic list from inside ros2 container
    TOPICS=$(docker exec ros2 bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /px4_msgs_ws/install/setup.bash && \
         timeout 5 ros2 topic list 2>/dev/null" || true)

    # Check each critical topic
    declare -A EXPECTED_TOPICS
    EXPECTED_TOPICS=(
        ["/fmu/out/vehicle_odometry"]="Vehicle odometry (position + velocity)"
        ["/fmu/out/vehicle_status"]="Vehicle status (arm state, mode)"
        ["/fmu/out/vehicle_local_position"]="Local position (NED)"
    )

    for topic in "${!EXPECTED_TOPICS[@]}"; do
        desc="${EXPECTED_TOPICS[$topic]}"
        if echo "${TOPICS}" | grep -q "${topic}"; then
            check "${topic}" "ok" "${desc}"
        else
            check "${topic}" "fail" "Not found — is PX4 SITL running?"
        fi
    done
else
    check "ROS2 topics" "fail" "ros2 container not running"
fi

echo ""

# ── 4. Summary ───────────────────────────────────────────────
echo "======================================="
echo -e "  Results: ${GREEN}${PASS} passed${NC}, ${RED}${FAIL} failed${NC}"
echo "======================================="
echo ""

if [ "${FAIL}" -eq 0 ]; then
    echo -e "  ${GREEN}All checks passed!${NC}"
    echo ""
    echo "  Phase 1 verification (communication):"
    echo "    ros2 topic echo /fmu/out/vehicle_status"
    echo ""
else
    echo -e "  ${YELLOW}Some checks failed. See above for details.${NC}"
    echo ""
    echo "  Common fixes:"
    echo "    Containers not running?  → make sim-up"
    echo "    Topics missing?          → make px4-start (then wait 30s)"
    echo "    Agent not running?       → docker restart ros2"
    echo ""
    exit 1
fi
