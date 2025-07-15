#!/bin/bash

# Handmatige Push Mapping Script
# Duw/trek de robot fysiek met je handen, SLAM mapt automatisch

echo "🤲 Handmatige Push Mapping"
echo "=========================="
echo "Duwschema:"
echo "1. Zet de robot aan (./start_robot.sh)"
echo "2. Start dit script"
echo "3. Duw/trek de robot fysiek met je handen"
echo "4. SLAM mapt automatisch op basis van LiDAR"
echo "5. Druk Ctrl+C om te stoppen"
echo ""

# Functie om de robot te stoppen bij exit
cleanup() {
    echo ""
    echo "🛑 Stoppen van handmatige mapping..."
    echo "✅ Mapping blijft actief. De map wordt opgeslagen door SLAM toolbox."
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

# Schakel wheel odometry uit voor betere LiDAR tracking
echo "🔧 Configureren voor LiDAR-only mapping..."
echo "🔄 Schakel wheel odometry uit voor betere LiDAR tracking..."
ros2 param set /diff_cont enable_odom_tf false

echo ""
echo "🎯 Klaar voor handmatige mapping!"
echo "================================"
echo "📋 Instructies:"
echo "1. Duw de robot voorzichtig vooruit"
echo "2. Trek de robot naar achteren"
echo "3. Draai de robot langzaam"
echo "4. Ga door alle hoeken en gangen"
echo "5. SLAM mapt automatisch op basis van LiDAR scans"
echo ""
echo "💡 Tips voor goede mapping:"
echo "- Beweeg langzaam en vloeiend"
echo "- Ga door alle ruimtes"
echo "- Maak volledige rondes"
echo "- Vermijd snelle bewegingen"
echo ""
echo "📊 Monitoring:"
echo "- Bekijk de map: ros2 topic echo /map"
echo "- Monitor LiDAR: ros2 topic echo /scan"
echo "- Check positie: ros2 topic echo /odom"
echo ""
echo "🔄 Mapping is actief... Duw de robot rond!"
echo "Druk Ctrl+C om te stoppen"

# Wacht tot gebruiker stopt
while true; do
    sleep 1
done 