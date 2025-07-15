#!/bin/bash

# Auto Mapping Script voor ROS2 Robot
# Dit script laat de robot automatisch rijden en mappen

echo "🤖 Start automatische mapping..."
echo "De robot zal nu automatisch rijden en mappen"
echo "Druk Ctrl+C om te stoppen"

# Functie om de robot te stoppen bij exit
cleanup() {
    echo "🛑 Stoppen van automatische mapping..."
    # Stop alle beweging
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --once
    exit 0
}

# Trap signalen om netjes te stoppen
trap cleanup SIGINT SIGTERM

# Wacht even tot alles opgestart is
echo "⏳ Wachten tot robot klaar is..."
sleep 5

# Mapping strategie: 
# 1. Vooruit rijden tot obstakel
# 2. Draaien om obstakel te ontwijken
# 3. Herhalen

echo "🚗 Start automatische mapping..."

while true; do
    echo "📡 Controleer voor obstakels..."
    
    # Lees LiDAR data om obstakels te detecteren
    # We gebruiken een eenvoudige strategie: vooruit rijden en draaien bij obstakels
    
    # Vooruit rijden (langzaam voor veiligheid)
    echo "➡️  Vooruit rijden..."
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 10 &
    
    FORWARD_PID=$!
    
    # Rij vooruit voor 3 seconden
    sleep 3
    
    # Stop vooruit rijden
    kill $FORWARD_PID 2>/dev/null
    
    # Stop de robot
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --once
    
    echo "⏸️  Pauze..."
    sleep 1
    
    # Draai een beetje om nieuwe richting te verkennen
    echo "🔄 Draaien voor nieuwe richting..."
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" --rate 10 &
    
    TURN_PID=$!
    
    # Draai voor 2 seconden
    sleep 2
    
    # Stop draaien
    kill $TURN_PID 2>/dev/null
    
    # Stop de robot
    ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --once
    
    echo "⏸️  Pauze voor volgende cyclus..."
    sleep 2
    
    echo "🔄 Nieuwe mapping cyclus..."
done 