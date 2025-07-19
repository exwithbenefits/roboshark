#!/bin/bash

# 🧪 Test Autonomous Mapping System
# Quick test to verify all components are working

echo "🧪 Testing Autonomous Mapping System"
echo "===================================="

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

echo "🔍 System Check:"
echo "================"

# Test 1: Check if robot system is running
echo -n "1. Robot system: "
if ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "✅ Running"
else
    echo "❌ Not running (start with ./start_robot.sh)"
    exit 1
fi

# Test 2: Check LiDAR data
echo -n "2. LiDAR data: "
if timeout 3 ros2 topic echo /scan --once 2>/dev/null | grep -q "ranges"; then
    echo "✅ Available"
else
    echo "❌ No data"
    exit 1
fi

# Test 3: Check motor control
echo -n "3. Motor control: "
if ros2 topic list 2>/dev/null | grep -q "/diff_cont/cmd_vel_unstamped"; then
    echo "✅ Available"
else
    echo "❌ Not available"
    exit 1
fi

# Test 4: Check SLAM mapping
echo -n "4. SLAM mapping: "
if ros2 topic list 2>/dev/null | grep -q "/map"; then
    echo "✅ Running"
else
    echo "❌ Not running"
    exit 1
fi

# Test 5: Check Python dependencies
echo -n "5. Python deps: "
if python3 -c "import numpy; import rclpy" 2>/dev/null; then
    echo "✅ Available"
else
    echo "❌ Missing dependencies"
    echo "   Install with: pip3 install numpy"
    exit 1
fi

echo ""
echo "🎯 Quick Movement Test:"
echo "======================"
echo "Testing robot movement for 3 seconds..."

# Test forward movement
echo "Moving forward..."
timeout 2 ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear: {x: 0.1}" --rate 10 &

sleep 2

# Stop robot
echo "Stopping robot..."
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear: {x: 0.0}" --once

echo ""
echo "✅ All tests passed!"
echo "🚀 System is ready for autonomous mapping"
echo ""
echo "To start autonomous mapping, run: ./autonomous_mapping.sh" 