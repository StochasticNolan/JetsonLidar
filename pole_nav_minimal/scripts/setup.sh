#!/bin/bash
# One-time setup for Jetson

set -e

echo "=== Pole Navigation Setup ==="

# Check if on Jetson
if [ -f /etc/nv_tegra_release ]; then
    echo "✓ Jetson detected"
    IS_JETSON=true
else
    echo "⚠ Not a Jetson - some optimizations may not apply"
    IS_JETSON=false
fi

# Update system
echo "Updating system..."
sudo apt-get update

# Install system dependencies
echo "Installing system dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    git

# Upgrade pip
echo "Upgrading pip..."
pip3 install --upgrade pip

# Install Python packages
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Create data directories
echo "Creating directories..."
sudo mkdir -p /data/screenshots
sudo mkdir -p /data/models
sudo chown -R $USER:$USER /data

# Check for model
if [ ! -f "models/best.pt" ]; then
    echo ""
    echo "⚠ WARNING: YOLO model not found!"
    echo "Copy your model: cp /path/to/best.pt models/"
    echo ""
fi

# Jetson-specific optimizations
if [ "$IS_JETSON" = true ]; then
    echo "Applying Jetson optimizations..."

    # Enable max performance mode
    sudo nvpmodel -m 0
    sudo jetson_clocks

    echo "✓ Performance mode enabled"
fi

# Test imports
echo "Testing Python imports..."
python3 -c "import cv2; import numpy; import yaml; from ultralytics import YOLO; print('✓ All imports successful')"

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Copy your YOLO model to models/best.pt"
echo "2. Edit config/settings.yaml (especially LiDAR IP)"
echo "3. Run: ./scripts/run.sh"
