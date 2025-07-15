#!/bin/bash

# Robot Auto Startup Script
# Dit script start alle componenten voor de robot met LiDAR en SLAM

echo "🤖 Starting Robot Auto System..."
echo "=================================="

# Source bashrc for environment variables and aliases
source /root/.bashrc

# Source ROS2 environment
#source /opt/ros/humble/setup.bash
#source /sbx/ros2/install/setup.bash

# Functie om te checken of een topic bestaat
check_topic() {
    local topic_name=$1
    local timeout=10
    local count=0
    
    echo "Checking for topic: $topic_name"
    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "$topic_name"; then
            echo "✅ Topic $topic_name found!"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    echo "❌ Topic $topic_name not found after $timeout seconds"
    return 1
}

# Functie om te checken of een node draait
check_node() {
    local node_name=$1
    local timeout=10
    local count=0
    
    echo "Checking for node: $node_name"
    while [ $count -lt $timeout ]; do
        if ros2 node list | grep -q "$node_name"; then
            echo "✅ Node $node_name is running!"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    echo "❌ Node $node_name not found after $timeout seconds"
    return 1
}

# Stop alle bestaande ROS2 processen
echo "🔄 Stopping any existing ROS2 processes..."
pkill -f "ros2 launch" 2>/dev/null
pkill -f "ros2_control_node" 2>/dev/null
pkill -f "rplidar_composition" 2>/dev/null
pkill -f "controller_manager" 2>/dev/null
pkill -f "spawner" 2>/dev/null
sleep 5

# Start de hoofd launch file
echo "🚀 Starting robot with LiDAR and SLAM..."
ros2 launch articubot_one real_mapping.launch.py &
ROBOT_PID=$!

# Wacht op initialisatie
echo "⏳ Waiting for robot components to start..."
sleep 8

# Check of essentiële componenten draaien
echo "🔍 Checking system status..."

# Check essentiële topics
check_topic "/scan" || echo "⚠️  LiDAR scan topic not found"
check_topic "/odom" || echo "⚠️  Odometry topic not found"
check_topic "/map" || echo "⚠️  SLAM map topic not found"
check_topic "/robot_description" || echo "⚠️  Robot description topic not found"

# Check essentiële nodes
check_node "rplidar_node" || echo "⚠️  LiDAR node not found"
check_node "slam_toolbox" || echo "⚠️  SLAM node not found"
check_node "controller_manager" || echo "⚠️  Controller manager not found"

echo ""
echo "🎯 Robot System Status:"
echo "========================="
echo "📡 Active Topics:"
ros2 topic list 2>/dev/null | head -10 || echo "Topics not available"

echo ""
echo "🔧 Active Nodes:"
ros2 node list 2>/dev/null || echo "Nodes not available"

echo ""
echo "📊 System Information:"
echo "- LiDAR: $(timeout 5 ros2 topic echo /scan --once 2>/dev/null | grep -c 'ranges' || echo 'Not available')"
echo "- Map Resolution: $(timeout 5 ros2 topic echo /map --once 2>/dev/null | grep 'resolution' | head -1 || echo 'Not available')"
echo "- Robot Position: $(timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep 'position' | head -1 || echo 'Not available')"

echo ""
echo "🔧 Monitoring Commands:"
echo "- View all topics: ros2 topic list"
echo "- Monitor LiDAR: ros2 topic echo /scan"
echo "- Monitor position: ros2 topic echo /odom"
echo "- Monitor map: ros2 topic echo /map"

echo ""
echo "✅ Robot system startup complete!"
echo "Press Ctrl+C to stop all processes"

# Stop alles netjes bij Ctrl+C
trap 'echo ""; echo "🛑 Stopping robot system..."; kill $ROBOT_PID 2>/dev/null; pkill -f "ros2 launch" 2>/dev/null; echo "✅ Robot system stopped."; exit 0' INT

# Script blijft draaien
while true; do
    sleep 1
done 
