#!/bin/bash

# RViz Mapping Visualisatie Script
# Start RViz met configuratie voor LiDAR en SLAM mapping

echo "🗺️  RViz Mapping Visualisatie"
echo "=============================="
echo "Start RViz met LiDAR en SLAM visualisatie"
echo ""

# Controleer of robot system draait
echo "🔍 Controleren of robot system actief is..."
if ! ros2 topic list | grep -q "/scan"; then
    echo "❌ Robot system niet actief! Start eerst ./start_robot.sh"
    exit 1
fi

echo "✅ Robot system is actief"
echo "📡 LiDAR topic: /scan"
echo "🗺️  SLAM topic: /map"
echo ""

# Start RViz met mapping configuratie
echo "🚀 Start RViz voor mapping visualisatie..."
echo ""

# Start RViz met specifieke configuratie voor mapping
rviz2 -d /sbx/ros2/src/articubot_one/config/mapping.rviz 2>/dev/null || \
rviz2 -d /sbx/ros2/install/articubot_one/share/articubot_one/config/mapping.rviz 2>/dev/null || \
rviz2

echo ""
echo "📋 RViz Instructies:"
echo "===================="
echo "1. Als RViz opent, voeg deze displays toe:"
echo "   - Add → By topic → /scan → LaserScan"
echo "   - Add → By topic → /map → Map"
echo "   - Add → By topic → /odom → PoseArray"
echo "   - Add → TF (voor robot frame)"
echo ""
echo "2. Configureer displays:"
echo "   - LaserScan: Topic = /scan"
echo "   - Map: Topic = /map"
echo "   - TF: Frames = base_link, laser_frame"
echo ""
echo "3. Nu zie je:"
echo "   - 🔴 LiDAR scans (rode punten)"
echo "   - 🗺️  SLAM map (grijze vlakken)"
echo "   - 🤖 Robot positie (groene pijl)"
echo ""
echo "4. Duw de robot rond en zie de map groeien!"
echo ""
echo "Druk Ctrl+C om RViz te sluiten" 