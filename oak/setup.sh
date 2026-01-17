#!/bin/bash
# Setup script for OAK-D Pro W PoE camera
# Run with: ./oak/setup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=== OAK-D Pro W PoE Setup ==="
echo

# 1. Check/configure network
CAMERA_SUBNET="192.168.18"
CAMERA_IP="${CAMERA_SUBNET}.103"
PC_IP="${CAMERA_SUBNET}.1"

echo "[1/3] Checking network configuration..."

# Find ethernet interface (first non-loopback, non-wireless, non-docker UP interface)
ETH_IFACE=$(ip -o link show | grep -v "lo:\|wl\|docker\|virbr" | grep "state UP" | head -1 | awk -F': ' '{print $2}')

if [ -z "$ETH_IFACE" ]; then
    echo -e "${RED}ERROR: No ethernet interface found. Connect ethernet cable.${NC}"
    exit 1
fi

echo "  Ethernet interface: $ETH_IFACE"

# Check if we already have the camera subnet IP
if ip addr show "$ETH_IFACE" | grep -q "${CAMERA_SUBNET}\."; then
    echo -e "  ${GREEN}✓ Already have IP in ${CAMERA_SUBNET}.x subnet${NC}"
else
    echo -e "  ${YELLOW}Adding ${PC_IP}/24 to ${ETH_IFACE}...${NC}"
    echo "  (requires sudo)"
    sudo ip addr add ${PC_IP}/24 dev "$ETH_IFACE" 2>/dev/null || true
    echo -e "  ${GREEN}✓ IP added${NC}"
fi

# Test camera connectivity
echo "  Testing camera at ${CAMERA_IP}..."
if ping -c 1 -W 2 "$CAMERA_IP" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓ Camera reachable${NC}"
else
    echo -e "  ${RED}✗ Camera not reachable at ${CAMERA_IP}${NC}"
    echo "    Check: cable connected? switch powered?"
    exit 1
fi

# 2. Download models if missing
echo
echo "[2/3] Checking models..."

MODELS_DIR="$SCRIPT_DIR/models"
CUSTOM_MODELS_DIR="$SCRIPT_DIR/custom_models"

mkdir -p "$MODELS_DIR" "$CUSTOM_MODELS_DIR"

GITHUB_RAW="https://github.com/geaxgx/depthai_blazepose/raw/main"

download_if_missing() {
    local file="$1"
    local url="$2"

    if [ -f "$file" ]; then
        echo -e "  ${GREEN}✓ $(basename "$file")${NC}"
    else
        echo -e "  ${YELLOW}Downloading $(basename "$file")...${NC}"
        curl -sL "$url" -o "$file"
        echo -e "  ${GREEN}✓ $(basename "$file")${NC}"
    fi
}

download_if_missing "$MODELS_DIR/pose_detection_sh4.blob" \
    "$GITHUB_RAW/models/pose_detection_sh4.blob"

download_if_missing "$MODELS_DIR/pose_landmark_full_sh4.blob" \
    "$GITHUB_RAW/models/pose_landmark_full_sh4.blob"

download_if_missing "$MODELS_DIR/pose_landmark_lite_sh4.blob" \
    "$GITHUB_RAW/models/pose_landmark_lite_sh4.blob"

download_if_missing "$CUSTOM_MODELS_DIR/DetectionBestCandidate_sh1.blob" \
    "$GITHUB_RAW/custom_models/DetectionBestCandidate_sh1.blob"

download_if_missing "$CUSTOM_MODELS_DIR/DivideBy255_sh1.blob" \
    "$GITHUB_RAW/custom_models/DivideBy255_sh1.blob"

# 3. Update .env
echo
echo "[3/3] Checking .env..."

ENV_FILE="$PROJECT_DIR/.env"

if [ -f "$ENV_FILE" ] && grep -q "CAMERA_IP=${CAMERA_IP}" "$ENV_FILE"; then
    echo -e "  ${GREEN}✓ .env already configured${NC}"
else
    echo "CAMERA_IP=${CAMERA_IP}" > "$ENV_FILE"
    echo -e "  ${GREEN}✓ .env updated with CAMERA_IP=${CAMERA_IP}${NC}"
fi

echo
echo -e "${GREEN}=== Setup complete ===${NC}"
echo
echo "Run: python oak/blazepose_live.py"
