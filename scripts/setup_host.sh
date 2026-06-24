#!/bin/bash
# =============================================================
# scripts/setup_host.sh
# =============================================================
# One-shot host prerequisite installer.
# Run this ONCE on a fresh Ubuntu 22.04 machine.
#
# What it does:
#   1. Installs Docker Engine + docker-compose-plugin
#   2. Adds current user to docker group
#   3. Sets up X11 forwarding for Gazebo GUI (optional)
#   4. Creates/verifies the .env file
#   5. Checks system requirements
#
# Usage:
#   bash scripts/setup_host.sh
# =============================================================

set -e  # Exit on any error

# ── Colors for output ─────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()    { echo -e "${BLUE}[INFO]${NC}  $1"; }
success() { echo -e "${GREEN}[OK]${NC}    $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC}  $1"; }
error()   { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

echo ""
echo "=============================================="
echo "  GPS-Denied Drone Stack — Host Setup"
echo "=============================================="
echo ""

# ── Check OS ─────────────────────────────────────────────────
info "Checking operating system..."
if ! lsb_release -d 2>/dev/null | grep -q "Ubuntu 22.04"; then
    warning "This script is designed for Ubuntu 22.04."
    warning "Detected: $(lsb_release -d 2>/dev/null | cut -f2)"
    warning "Continuing anyway — some steps may fail."
fi
success "OS check done."

# ── Check RAM ────────────────────────────────────────────────
info "Checking available RAM..."
TOTAL_RAM_GB=$(awk '/MemTotal/ { printf "%.0f", $2/1024/1024 }' /proc/meminfo)
if [ "${TOTAL_RAM_GB}" -lt 8 ]; then
    warning "Only ${TOTAL_RAM_GB}GB RAM detected. Recommended: 16GB+"
    warning "PX4 build + Gazebo may be very slow or fail."
else
    success "${TOTAL_RAM_GB}GB RAM — sufficient."
fi

# ── Check Disk Space ──────────────────────────────────────────
info "Checking disk space..."
FREE_GB=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
if [ "${FREE_GB}" -lt 30 ]; then
    error "Only ${FREE_GB}GB free. Need at least 30GB (Docker images + PX4 build ≈ 20GB)."
fi
success "${FREE_GB}GB free — sufficient."

# ── Install Docker ────────────────────────────────────────────
info "Checking Docker installation..."
if command -v docker &>/dev/null; then
    DOCKER_VERSION=$(docker --version | awk '{print $3}' | sed 's/,//')
    success "Docker already installed: ${DOCKER_VERSION}"
else
    info "Installing Docker Engine..."
    # Official Docker install script
    curl -fsSL https://get.docker.com | sudo bash
    success "Docker installed."
fi

# ── Add user to docker group ──────────────────────────────────
if groups "${USER}" | grep -q docker; then
    success "User '${USER}' already in docker group."
else
    info "Adding '${USER}' to docker group..."
    sudo usermod -aG docker "${USER}"
    warning "You must LOG OUT and LOG BACK IN for docker group to take effect."
    warning "Or run: newgrp docker"
fi

# ── Check docker compose plugin ───────────────────────────────
info "Checking docker compose plugin..."
if docker compose version &>/dev/null; then
    success "docker compose: $(docker compose version | head -1)"
else
    info "Installing docker compose plugin..."
    sudo apt-get update -qq
    sudo apt-get install -y docker-compose-plugin
    success "docker compose plugin installed."
fi

# ── X11 Forwarding (for Gazebo GUI) ──────────────────────────
info "Setting up X11 forwarding for Docker..."
if command -v xhost &>/dev/null; then
    xhost +local:docker 2>/dev/null && \
        success "xhost configured: local Docker allowed." || \
        warning "xhost failed — is DISPLAY set? (Not needed if using HEADLESS=1)"
else
    warning "xhost not found. Install with: sudo apt-get install x11-xserver-utils"
    warning "This is only needed if you want to see the Gazebo GUI window."
    warning "With PX4_HEADLESS=1 in .env, xhost is NOT required."
fi

# ── Verify .env file ──────────────────────────────────────────
info "Checking .env file..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"

if [ -f "${REPO_ROOT}/.env" ]; then
    success ".env file found."
else
    error ".env file missing. Run from the project root (px4_docker_ws/)."
fi

# ── BuildKit ─────────────────────────────────────────────────
info "Enabling Docker BuildKit (faster builds)..."
if ! grep -q "DOCKER_BUILDKIT" ~/.bashrc 2>/dev/null; then
    echo "export DOCKER_BUILDKIT=1" >> ~/.bashrc
    success "DOCKER_BUILDKIT=1 added to ~/.bashrc"
else
    success "DOCKER_BUILDKIT already set."
fi

# ── Summary ──────────────────────────────────────────────────
echo ""
echo "=============================================="
echo -e "${GREEN}  Host setup complete!${NC}"
echo "=============================================="
echo ""
echo "  Next steps:"
echo ""
echo "  1. If this is your first time:"
echo "       newgrp docker    (or log out and back in)"
echo ""
echo "  2. Build the Docker images (~35 min first time):"
echo "       make build"
echo ""
echo "  3. Start the simulation stack:"
echo "       make sim-up"
echo ""
echo "  4. Verify everything is working:"
echo "       make health"
echo ""
