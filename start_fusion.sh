#!/bin/bash
# Simple launcher for fusion_ros.py with Aeva Atlas lidar
# This script handles all the setup automatically

set -e

# Configuration
readonly PROJECT_DIR="/home/nvidia/Projects/JetsonLidar"
readonly AEVA_IP="10.5.50.100"
readonly AEVA_INTERFACE="eth1"
readonly AEVA_LOCAL_IP="10.5.50.1"
readonly AEVA_TOPIC="/aeva_0/points"
readonly TOPIC_TIMEOUT=20
readonly CAMERA_DEVICE="/dev/video0"
readonly CAMERA_EXPOSURE=1000

# Track PIDs for cleanup
declare -a PIDS=()
NEED_ROSCORE=false

cd "$PROJECT_DIR"

# Kill a process gracefully, then forcefully if needed
kill_process() {
    local pid=$1
    local name=$2
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        echo "  Stopping $name (PID: $pid)..."
        kill -TERM "$pid" 2>/dev/null || true
        sleep 0.5
        kill -KILL "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    echo ""
    echo "Cleaning up..."

    # Kill tracked processes in reverse order
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        kill_process "${PIDS[i]}" "process"
    done

    # Kill any remaining aeva_simple_bridge processes
    pkill -f "aeva_simple_bridge" 2>/dev/null || true

    # Kill roscore if we started it
    if [[ "$NEED_ROSCORE" == true ]]; then
        pkill -f "rosmaster" 2>/dev/null || true
        pkill -f "rosout" 2>/dev/null || true
    fi

    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Source ROS distribution
source_ros() {
    local ros_distros=("noetic" "melodic" "humble")
    for distro in "${ros_distros[@]}"; do
        local setup_file="/opt/ros/${distro}/setup.bash"
        if [[ -f "$setup_file" ]]; then
            source "$setup_file"
            echo "  ✓ ROS ${distro^} sourced"
            return 0
        fi
    done
    echo "  ✗ ERROR: ROS not found!"
    echo "  Please install ROS first: sudo apt install ros-noetic-desktop"
    exit 1
}

# Configure network interface for Aeva sensor
setup_aeva_network() {
    echo "  Checking ${AEVA_INTERFACE} interface..."

    # Check if interface exists
    if ! ip link show "$AEVA_INTERFACE" &>/dev/null; then
        echo "  ✗ ERROR: Interface $AEVA_INTERFACE not found!"
        echo "  Available interfaces:"
        ip link show | grep -E "^[0-9]+:" | awk '{print "    " $2}'
        return 1
    fi

    # Check if cable is connected (carrier)
    if ! ip link show "$AEVA_INTERFACE" | grep -q "state UP"; then
        echo "  ○ Bringing up $AEVA_INTERFACE..."
        sudo ip link set "$AEVA_INTERFACE" up
        sleep 1
    fi

    # Check carrier again
    if ip link show "$AEVA_INTERFACE" | grep -q "NO-CARRIER"; then
        echo "  ⚠ WARNING: No cable detected on $AEVA_INTERFACE"
        echo "  Please check the Ethernet cable to the Aeva sensor"
        return 1
    fi

    # Check if IP is already configured
    if ip addr show "$AEVA_INTERFACE" | grep -q "inet ${AEVA_LOCAL_IP}"; then
        echo "  ✓ $AEVA_INTERFACE already configured (${AEVA_LOCAL_IP})"
    else
        echo "  ○ Configuring $AEVA_INTERFACE with ${AEVA_LOCAL_IP}/24..."
        sudo ip addr add "${AEVA_LOCAL_IP}/24" dev "$AEVA_INTERFACE" 2>/dev/null || true
        echo "  ✓ $AEVA_INTERFACE configured"
    fi

    # Verify connectivity
    echo -n "  Testing connection to Aeva ($AEVA_IP)..."
    if ping -c 1 -W 2 "$AEVA_IP" &>/dev/null; then
        echo " ✓ OK"
        return 0
    else
        echo " ✗ FAILED"
        echo "  Cannot reach Aeva sensor at $AEVA_IP"
        return 1
    fi
}

# Set camera exposure and autofocus using v4l2-ctl (more reliable)
set_camera_settings() {
    # Try v4l2-ctl first (most reliable)
    if command -v v4l2-ctl &>/dev/null; then
        # Enable autofocus (continuous)
        v4l2-ctl -d "$CAMERA_DEVICE" --set-ctrl=focus_automatic_continuous=1 2>/dev/null || true
        # Set manual exposure mode and value
        v4l2-ctl -d "$CAMERA_DEVICE" --set-ctrl=auto_exposure=1 2>/dev/null || true
        v4l2-ctl -d "$CAMERA_DEVICE" --set-ctrl=exposure_time_absolute=${CAMERA_EXPOSURE} 2>/dev/null || true
        echo "  ✓ Camera: exposure=${CAMERA_EXPOSURE}, autofocus=ON (v4l2-ctl)"
    else
        # Fallback to Python/ioctl
        python3 << EOF
import fcntl
import struct
try:
    V4L2_CID_EXPOSURE_AUTO = 0x009a0901
    V4L2_CID_EXPOSURE_ABSOLUTE = 0x009a0902
    V4L2_CID_FOCUS_AUTO = 0x009a090c
    VIDIOC_S_CTRL = 0xc008561c
    with open('${CAMERA_DEVICE}', 'rb+', buffering=0) as fd:
        fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack('Ii', V4L2_CID_EXPOSURE_AUTO, 1))
        fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack('Ii', V4L2_CID_EXPOSURE_ABSOLUTE, ${CAMERA_EXPOSURE}))
        fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack('Ii', V4L2_CID_FOCUS_AUTO, 1))
    print("  ✓ Camera: exposure=${CAMERA_EXPOSURE}, autofocus=ON")
except Exception as e:
    print(f"  ⚠ Could not set camera settings: {e}")
EOF
    fi
}

# Wait for ROS topic with timeout
wait_for_topic() {
    local topic=$1
    local timeout=$2
    local elapsed=0

    while ! rostopic list 2>/dev/null | grep -q "$topic"; do
        if ((elapsed >= timeout)); then
            echo ""
            echo "  ✗ ERROR: $topic topic not available after ${timeout}s"
            echo ""
            echo "Please check:"
            echo "  1. Aeva sensor is powered on and in APP_ON state"
            echo "  2. Network connection: ping $AEVA_IP"
            echo "  3. Aeva web interface: http://$AEVA_IP/"
            echo "  4. Sensor is actively sensing (check for lens cover)"
            exit 1
        fi
        sleep 1
        ((elapsed++))
        echo -n "."
    done
    echo ""
}

# ==========================================
# Main execution
# ==========================================

echo "=========================================="
echo "Aeva Atlas + Fusion Launcher"
echo "=========================================="

# Step 1: Source ROS
echo ""
echo "[1/7] Setting up ROS..."
source_ros

# Source Aeva ROS workspace
local_ws="${PROJECT_DIR}/ros_examples/devel/setup.bash"
if [[ -f "$local_ws" ]]; then
    source "$local_ws"
    echo "  ✓ Aeva ROS workspace sourced"
else
    echo "  ✗ ERROR: Aeva ROS workspace not built!"
    echo "  Please build it first: cd ros_examples && catkin_make"
    exit 1
fi

# Step 2: Configure network for Aeva sensor
echo ""
echo "[2/7] Configuring network for Aeva sensor..."
if ! setup_aeva_network; then
    echo ""
    echo "  ⚠ Aeva network setup failed - LiDAR may not work"
    echo "  Continuing with camera only..."
    echo ""
fi

# Step 3: Check/start roscore
echo ""
echo "[3/7] Checking ROS master..."
if rostopic list &>/dev/null; then
    echo "  ✓ ROS master is running"
else
    echo "  ○ ROS master not running, starting roscore..."
    roscore &
    PIDS+=($!)
    NEED_ROSCORE=true
    sleep 3
    echo "  ✓ roscore started (PID: ${PIDS[-1]})"
fi

# Step 4: Start Aeva bridge
echo ""
echo "[4/7] Starting Aeva ROS bridge..."
echo "  Connecting to Aeva at $AEVA_IP"
echo "  Using FULL FRAME compensated point cloud (like web viewer)"
"${PROJECT_DIR}/build/aeva_simple_bridge" "$AEVA_IP" aeva &
PIDS+=($!)
echo "  ✓ Bridge started (PID: ${PIDS[-1]})"

# Step 5: Wait for topic
echo ""
echo "[5/7] Waiting for $AEVA_TOPIC topic..."
wait_for_topic "$AEVA_TOPIC" "$TOPIC_TIMEOUT"
echo "  ✓ $AEVA_TOPIC topic is available!"

# Check for data
echo "  Checking for data..."
if timeout 5 rostopic echo "$AEVA_TOPIC" -n 1 &>/dev/null; then
    echo "  ✓ Receiving point cloud data with improved 3D viz!"
else
    echo "  ⚠ Topic exists but no data yet"
    echo "  The sensor may not be actively sensing"
    echo "  Continuing anyway..."
fi

# Step 6: Set camera settings
echo ""
echo "[6/7] Setting camera settings..."
set_camera_settings

# Step 7: Run fusion
echo ""
echo "[7/7] Starting fusion_ros.py..."
echo "=========================================="
echo "Controls: 'f'=front view, 'r'=reset (overhead), 'p'=point size, 'q'=quit"
echo "=========================================="
echo ""

python3 fusion_ros.py &
PIDS+=($!)
echo "  Fusion started (PID: ${PIDS[-1]})"

# Wait for fusion to exit
wait "${PIDS[-1]}" 2>/dev/null || true
