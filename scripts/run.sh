#!/bin/bash
# Launch pole navigation

set -e

echo "=== Starting Pole Navigation ==="

# Check model exists
if [ ! -f "models/best.pt" ]; then
    echo "ERROR: Model not found at models/best.pt"
    echo "Copy your model first: cp /path/to/best.pt models/"
    exit 1
fi

# Check config
if [ ! -f "config/settings.yaml" ]; then
    echo "ERROR: Config not found at config/settings.yaml"
    exit 1
fi

# Create output directory
mkdir -p /data/screenshots

# Run fusion script
echo "Launching fusion..."
python3 fusion.py

echo ""
echo "=== Navigation Stopped ==="
