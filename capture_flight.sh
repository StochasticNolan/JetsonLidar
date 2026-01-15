#!/bin/bash
# Flight data capture script for offline development
# Records LiDAR point clouds and camera images via rosbag

set -e

# Configuration
readonly PROJECT_DIR="/home/nvidia/Projects/JetsonLidar"
readonly AEVA_IP="10.5.50.100"
readonly AEVA_TOPIC="/aeva_0/points"
readonly CAMERA_TOPIC="/flight_cam/image_raw"
readonly CAMERA_DEVICE="/dev/video1"
readonly CAPTURE_DIR="${PROJECT_DIR}/flight_captures"
readonly TOPIC_TIMEOUT=20

# Options (can be overridden via command line)
RECORD_CAMERA=true
COMPRESS_BAG=false

# Track PIDs for cleanup
declare -a PIDS=()
NEED_ROSCORE=false
ROSBAG_PID=""
SESSION_DIR=""

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --no-camera     Skip camera recording (LiDAR only)"
    echo "  --compress      Compress rosbag (slower, smaller files)"
    echo "  -h, --help      Show this help"
    echo ""
    echo "Captures are saved to: $CAPTURE_DIR/<timestamp>/"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-camera)
            RECORD_CAMERA=false
            shift
            ;;
        --compress)
            COMPRESS_BAG=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

cd "$PROJECT_DIR"

kill_process() {
    local pid=$1
    local name=$2
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        echo "  Stopping $name (PID: $pid)..."
        kill -INT "$pid" 2>/dev/null || true
        sleep 1
        kill -TERM "$pid" 2>/dev/null || true
        sleep 0.5
        kill -KILL "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    echo ""
    echo "=========================================="
    echo "Stopping capture..."
    echo "=========================================="

    # Stop rosbag first (INT signal to finish writing)
    if [[ -n "$ROSBAG_PID" ]] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "  Finishing rosbag write..."
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        # Wait for rosbag to finish writing (up to 10 seconds)
        for i in {1..10}; do
            if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        kill -KILL "$ROSBAG_PID" 2>/dev/null || true
    fi

    # Kill other processes in reverse order
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        kill_process "${PIDS[i]}" "process"
    done

    pkill -f "aeva_simple_bridge" 2>/dev/null || true
    pkill -f "usb_cam_node" 2>/dev/null || true

    if [[ "$NEED_ROSCORE" == true ]]; then
        pkill -f "rosmaster" 2>/dev/null || true
        pkill -f "rosout" 2>/dev/null || true
    fi

    # Show capture summary
    if [[ -n "$SESSION_DIR" ]] && [[ -d "$SESSION_DIR" ]]; then
        echo ""
        echo "=========================================="
        echo "Capture complete!"
        echo "=========================================="
        echo "Location: $SESSION_DIR"
        echo ""
        echo "Files:"
        ls -lh "$SESSION_DIR"/*.bag 2>/dev/null || echo "  (no bag files)"
        echo ""
        echo "Playback command:"
        echo "  rosbag play $SESSION_DIR/*.bag"
        echo ""

        # Calculate total size
        local total_size
        total_size=$(du -sh "$SESSION_DIR" 2>/dev/null | cut -f1)
        echo "Total size: $total_size"
    fi

    echo ""
    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

source_ros() {
    local ros_distros=("noetic" "melodic" "humble")
    for distro in "${ros_distros[@]}"; do
        local setup_file="/opt/ros/${distro}/setup.bash"
        if [[ -f "$setup_file" ]]; then
            source "$setup_file"
            echo "  ROS ${distro^} sourced"
            return 0
        fi
    done
    echo "ERROR: ROS not found!"
    exit 1
}

wait_for_topic() {
    local topic=$1
    local timeout=$2
    local elapsed=0

    while ! rostopic list 2>/dev/null | grep -q "$topic"; do
        if ((elapsed >= timeout)); then
            return 1
        fi
        sleep 1
        ((elapsed++))
        echo -n "."
    done
    echo ""
    return 0
}

# ==========================================
# Main execution
# ==========================================

echo "=========================================="
echo "Flight Data Capture"
echo "=========================================="
echo "Camera recording: $RECORD_CAMERA"
echo "Compression: $COMPRESS_BAG"
echo ""

# Create session directory with timestamp
SESSION_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SESSION_DIR="${CAPTURE_DIR}/${SESSION_TIMESTAMP}"
mkdir -p "$SESSION_DIR"
echo "Session directory: $SESSION_DIR"
echo ""

# Step 1: Source ROS
echo "[1/5] Setting up ROS..."
source_ros

local_ws="${PROJECT_DIR}/ros_examples/devel/setup.bash"
if [[ -f "$local_ws" ]]; then
    source "$local_ws"
    echo "  Aeva workspace sourced"
else
    echo "ERROR: Aeva ROS workspace not built!"
    exit 1
fi

# Step 2: Check/start roscore
echo ""
echo "[2/5] Checking ROS master..."
if rostopic list &>/dev/null; then
    echo "  ROS master is running"
else
    echo "  Starting roscore..."
    roscore &
    PIDS+=($!)
    NEED_ROSCORE=true
    sleep 3
fi

# Step 3: Start Aeva bridge
echo ""
echo "[3/5] Starting Aeva LiDAR bridge..."
"${PROJECT_DIR}/build/aeva_simple_bridge" "$AEVA_IP" aeva &
PIDS+=($!)

echo -n "  Waiting for $AEVA_TOPIC"
if ! wait_for_topic "$AEVA_TOPIC" "$TOPIC_TIMEOUT"; then
    echo ""
    echo "ERROR: LiDAR topic not available. Is the Aeva sensor connected?"
    exit 1
fi
echo "  LiDAR ready"

# Step 4: Start camera node (optional)
echo ""
echo "[4/5] Setting up camera..."
TOPICS_TO_RECORD="$AEVA_TOPIC"

if [[ "$RECORD_CAMERA" == true ]]; then
    # Check if usb_cam is available
    if rospack find usb_cam &>/dev/null; then
        echo "  Starting usb_cam node..."
        rosrun usb_cam usb_cam_node \
            _video_device:="$CAMERA_DEVICE" \
            _image_width:=1280 \
            _image_height:=720 \
            _framerate:=30 \
            _pixel_format:=yuyv \
            __name:=flight_cam &
        PIDS+=($!)

        echo -n "  Waiting for camera topic"
        if wait_for_topic "$CAMERA_TOPIC" 10; then
            TOPICS_TO_RECORD="$AEVA_TOPIC $CAMERA_TOPIC"
            echo "  Camera ready"
        else
            echo ""
            echo "  WARNING: Camera topic not available, recording LiDAR only"
        fi
    else
        echo "  WARNING: usb_cam not installed"
        echo "  Install with: sudo apt install ros-noetic-usb-cam"
        echo "  Recording LiDAR only"
    fi
else
    echo "  Skipping camera (--no-camera flag)"
fi

# Step 5: Start recording
echo ""
echo "[5/5] Starting rosbag recording..."
echo "  Topics: $TOPICS_TO_RECORD"
echo ""

BAG_ARGS="-o ${SESSION_DIR}/flight"
if [[ "$COMPRESS_BAG" == true ]]; then
    BAG_ARGS="$BAG_ARGS --lz4"
    echo "  Compression: LZ4 enabled"
fi

# Save session metadata
cat > "${SESSION_DIR}/metadata.txt" << EOF
Flight Capture Session
======================
Timestamp: $(date)
Topics: $TOPICS_TO_RECORD
Compression: $COMPRESS_BAG
Aeva IP: $AEVA_IP
Camera Device: $CAMERA_DEVICE
Camera Recording: $RECORD_CAMERA
EOF

echo "=========================================="
echo "RECORDING - Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Start rosbag record
rosbag record $BAG_ARGS $TOPICS_TO_RECORD &
ROSBAG_PID=$!

echo "Recording started (rosbag PID: $ROSBAG_PID)"
echo ""

# Show live status
while kill -0 "$ROSBAG_PID" 2>/dev/null; do
    # Get current bag size
    BAG_FILE=$(ls -t "${SESSION_DIR}"/*.bag.active 2>/dev/null | head -1 || ls -t "${SESSION_DIR}"/*.bag 2>/dev/null | head -1)
    if [[ -n "$BAG_FILE" ]]; then
        BAG_SIZE=$(du -h "$BAG_FILE" 2>/dev/null | cut -f1)
        DURATION=$((SECONDS))
        printf "\r  Recording: %02d:%02d  Size: %s  " $((DURATION/60)) $((DURATION%60)) "$BAG_SIZE"
    fi
    sleep 2
done
