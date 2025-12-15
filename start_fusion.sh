#!/bin/bash
# Simple launcher for fusion_ros.py with Aeva Atlas lidar
# This script handles all the setup automatically

set -e

cd /home/nvidia/Projects/JetsonLidar

echo "=========================================="
echo "Aeva Atlas + Fusion Launcher"
echo "=========================================="

# 1. Source ROS
echo ""
echo "[1/5] Setting up ROS..."
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "  ✓ ROS Noetic sourced"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
    echo "  ✓ ROS Melodic sourced"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✓ ROS Humble sourced"
else
    echo "  ✗ ERROR: ROS not found!"
    echo "  Please install ROS first: sudo apt install ros-noetic-desktop"
    exit 1
fi

# Source the Aeva ROS workspace
if [ -f /home/nvidia/Projects/JetsonLidar/ros_examples/devel/setup.bash ]; then
    source /home/nvidia/Projects/JetsonLidar/ros_examples/devel/setup.bash
    echo "  ✓ Aeva ROS workspace sourced"
else
    echo "  ✗ ERROR: Aeva ROS workspace not built!"
    echo "  Please build it first: cd ros_examples && catkin_make"
    exit 1
fi

# 2. Check if roscore is running
echo ""
echo "[2/5] Checking ROS master..."
if rostopic list &>/dev/null; then
    echo "  ✓ ROS master is running"
    NEED_ROSCORE=false
else
    echo "  ○ ROS master not running, starting roscore..."
    roscore &
    ROSCORE_PID=$!
    NEED_ROSCORE=true
    sleep 3
    echo "  ✓ roscore started (PID: $ROSCORE_PID)"
fi

# 3. Start simple Aeva ROS bridge
echo ""
echo "[3/5] Starting Aeva ROS bridge..."
echo "  Connecting to Aeva at 10.5.50.100"
echo "  Using FULL FRAME compensated point cloud (like web viewer)"
/home/nvidia/Projects/JetsonLidar/build/aeva_simple_bridge 10.5.50.100 aeva &
PUBLISHER_PID=$!
echo "  ✓ Bridge started (PID: $PUBLISHER_PID)"

# 4. Wait for topic to be available
echo ""
echo "[4/5] Waiting for /aeva_0/points topic..."
TIMEOUT=20
ELAPSED=0
while ! rostopic list | grep -q "/aeva_0/points"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "  ✗ ERROR: /aeva_0/points topic not available after ${TIMEOUT}s"
        echo ""
        echo "Please check:"
        echo "  1. Aeva sensor is powered on and in APP_ON state"
        echo "  2. Network connection: ping 10.5.50.100"
        echo "  3. Aeva web interface: http://10.5.50.100/"
        echo "  4. Sensor is actively sensing (check for lens cover)"
        echo ""
        echo "Killing background processes..."
        kill $PUBLISHER_PID 2>/dev/null || true
        [ "$NEED_ROSCORE" = true ] && kill $ROSCORE_PID 2>/dev/null || true
        exit 1
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done
echo ""
echo "  ✓ /aeva_0/points topic is available!"

# Check if data is being published
echo "  Checking for data..."
if timeout 5 rostopic echo /aeva_0/points -n 1 &>/dev/null; then
    echo "  ✓ Receiving point cloud data with improved 3D viz!"
else
    echo "  ⚠ Topic exists but no data yet"
    echo "  The sensor may not be actively sensing"
    echo "  Continuing anyway..."
fi

# 5. Run fusion
echo ""
echo "[5/5] Starting fusion_ros.py..."
echo "=========================================="
echo "Controls: 'q'=quit, 'p'=pause, 's'=screenshot"
echo "=========================================="
echo ""

# Trap Ctrl+C to cleanup
cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $PUBLISHER_PID 2>/dev/null || true
    [ "$NEED_ROSCORE" = true ] && kill $ROSCORE_PID 2>/dev/null || true
    echo "✓ Stopped"
    exit 0
}
trap cleanup SIGINT SIGTERM

# Run the fusion script
python3 fusion_ros.py

# Cleanup on normal exit
cleanup
