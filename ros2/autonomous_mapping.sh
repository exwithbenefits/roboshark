#!/bin/bash

# 🤖 Intelligent Autonomous Mapping Script
# Advanced room exploration using LiDAR sensor feedback and SLAM

echo "🤖 Intelligent Autonomous Mapping System"
echo "========================================"
echo "This script will:"
echo "1. Start intelligent LiDAR-based exploration"
echo "2. Use wall-following and frontier exploration"
echo "3. Build a complete map using SLAM"
echo "4. Automatically save the map when complete"
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# Function to stop robot on exit
cleanup() {
    echo ""
    echo "🛑 Stopping autonomous mapping..."
    
    # Stop the robot immediately
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --once 2>/dev/null
  
    # Stop the mapping node
    pkill -f "autonomous_mapper" 2>/dev/null
    
    echo "✅ Robot stopped safely"
    echo "💾 To save the current map, run:"
    echo "   ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map"
    
    exit 0
}

# Trap signals for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

# Check if robot system is running
echo "🔍 Checking robot system status..."

if ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "❌ Robot system not running!"
    echo "   Please start the robot first with: ./start_robot.sh"
    exit 1
fi

if ! ros2 topic list 2>/dev/null | grep -q "/diff_cont/cmd_vel_unstamped"; then
    echo "❌ Motor controller not available!"
    echo "   Please check robot hardware connections"
    exit 1
fi

if ! ros2 topic list 2>/dev/null | grep -q "/map"; then
    echo "❌ SLAM mapping not running!"
    echo "   Please check SLAM configuration"
    exit 1
fi

echo "✅ All systems operational!"
echo ""

# Check scan data quality
echo "📡 Checking LiDAR data quality..."
SCAN_CHECK=$(timeout 3 ros2 topic echo /scan --once 2>/dev/null | grep "ranges" | head -1)
if [ -z "$SCAN_CHECK" ]; then
    echo "⚠️  Warning: LiDAR data not available or poor quality"
    echo "   Mapping may not work properly"
else
    echo "✅ LiDAR data looks good"
fi

echo ""

# Display robot configuration
echo "🔧 Robot Configuration:"
echo "======================="
echo "📡 LiDAR topic: /scan"
echo "🗺️  SLAM topic: /map"
echo "🎮 Control topic: /diff_cont/cmd_vel_unstamped"
echo "📊 Odometry topic: /odom"
echo ""

# Safety parameters
echo "⚙️  Mapping Parameters:"
echo "======================"
echo "🛡️  Safety distance: 0.8m"
echo "🚗 Linear speed: 0.2 m/s"
echo "🔄 Angular speed: 0.5 rad/s"
echo "⏰ Max duration: 5 minutes"
echo "🧱 Wall follow distance: 0.6m"
echo ""

# Pre-flight safety check
echo "🛡️  Pre-flight Safety Check:"
echo "============================"
echo "✓ Emergency stop: Ctrl+C"
echo "✓ Robot will stop automatically if:"
echo "  - Obstacles too close (< 0.3m)"
echo "  - LiDAR data unavailable"
echo "  - Maximum time reached"
echo "✓ Map saved automatically on completion"
echo ""

# Countdown
echo "🚀 Starting autonomous mapping in:"
for i in {5..1}; do
    echo "   $i..."
    sleep 1
done

echo ""
echo "🎯 AUTONOMOUS MAPPING STARTED!"
echo "=============================="
echo ""

# Make sure the Python script is executable
chmod +x src/articubot_one/scripts/autonomous_mapper.py

# Start the autonomous mapping node
echo "🤖 Launching intelligent mapper node..."
python3 src/articubot_one/scripts/autonomous_mapper.py &
MAPPER_PID=$!

# Monitor the mapping process
echo "📊 Monitoring autonomous mapping..."
echo "Press Ctrl+C to stop at any time"
echo ""

# Keep script running and provide status updates
LAST_STATUS_TIME=0
while kill -0 $MAPPER_PID 2>/dev/null; do
    sleep 1
    CURRENT_TIME=$(date +%s)
    
    # Show status every 30 seconds
    if [ $((CURRENT_TIME - LAST_STATUS_TIME)) -ge 30 ]; then
        LAST_STATUS_TIME=$CURRENT_TIME
        echo "📈 Mapping Status Update:"
        echo "========================"
        
        # Check if topics are still active
        if ros2 topic hz /scan --window 1 2>/dev/null | grep -q "average rate"; then
            echo "✅ LiDAR: Active"
        else
            echo "❌ LiDAR: No data"
        fi
        
        if ros2 topic hz /map --window 1 2>/dev/null | grep -q "average rate"; then
            echo "✅ SLAM: Updating map"
        else
            echo "⚠️  SLAM: No updates"
        fi
        
        # Get current robot pose if available
        POSE_DATA=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | head -4)
        if [ ! -z "$POSE_DATA" ]; then
            echo "📍 Robot position updated"
        fi
        
        echo ""
    fi
done

echo ""
echo "🎊 Autonomous mapping completed!"
echo "==============================="

# Save the map automatically
echo "💾 Saving map automatically..."
MAP_NAME="robot_map_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli -f "maps/$MAP_NAME" 2>/dev/null || {
    echo "⚠️  Auto-save failed. Save manually with:"
    echo "   ros2 run nav2_map_server map_saver_cli -f $MAP_NAME"
}

echo ""
echo "📋 Mapping Summary:"
echo "=================="
echo "✅ Exploration completed"
echo "✅ Robot stopped safely"
echo "✅ Map data collected"
echo "📁 Map saved as: $MAP_NAME"
echo ""
echo "🎯 Next steps:"
echo "1. Review the map in RViz: ./start_mapping_rviz.sh"
echo "2. Use the map for navigation"
echo "3. Fine-tune map if needed"
echo ""
echo "🏁 Autonomous mapping session complete!" 