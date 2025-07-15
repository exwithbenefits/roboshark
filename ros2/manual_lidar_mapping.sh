#!/bin/bash

# Handmatige LiDAR Mapping Script
# Start keyboard control voor handmatige besturing tijdens mapping

echo "🗺️  Handmatige LiDAR Mapping"
echo "=============================="
echo "Dit script start keyboard control voor handmatige mapping"
echo "Gebruik de toetsen om de robot te besturen:"
echo ""
echo "⌨️  Keyboard Controls:"
echo "  i = vooruit"
echo "  , = achteruit" 
echo "  j = links draaien"
echo "  l = rechts draaien"
echo "  k = stoppen"
echo "  q/z = snelheid aanpassen"
echo "  e/c = draaisnelheid aanpassen"
echo ""
echo "Druk Ctrl+C om te stoppen"
echo ""

# Functie om de robot te stoppen bij exit
cleanup() {
    echo ""
    echo "🛑 Stoppen van handmatige mapping..."
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --once
    echo "✅ Robot gestopt. Mapping blijft actief."
    exit 0
}

trap cleanup SIGINT SIGTERM

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

# Start keyboard control
echo "⌨️  Start keyboard control..."
echo "Gebruik de toetsen om te rijden (in deze terminal):"
echo ""

# Start teleop_twist_keyboard met remapping naar het juiste topic
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped 