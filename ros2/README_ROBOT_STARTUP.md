# 🤖 Robot Auto Startup System

Dit systeem zorgt ervoor dat je robot automatisch start met alle componenten (LiDAR, SLAM, motor control) na een herstart van de Raspberry Pi.

## 📁 Bestanden

- `start_robot.sh` - Hoofdscript dat alle robot componenten start
- `robot-auto.service` - Systemd service bestand voor automatische startup
- `install_service.sh` - Installatie script voor de service

## 🚀 Snelle Start

### Optie 1: Handmatig starten
```bash
cd /sbx/ros2
./start_robot.sh
```

### Optie 2: Automatische startup installeren
```bash
cd /sbx/ros2
sudo ./install_service.sh
sudo systemctl start robot-auto
```

## 📋 Service Beheer

Na installatie kun je de service beheren met:

```bash
# Service status bekijken
sudo systemctl status robot-auto

# Service starten
sudo systemctl start robot-auto

# Service stoppen
sudo systemctl stop robot-auto

# Logs bekijken
sudo journalctl -u robot-auto -f

# Automatische startup uitschakelen
sudo systemctl disable robot-auto
```

## 🎮 Robot Besturing

Zodra het systeem draait, kun je de robot besturen met:

```bash
# Vooruit rijden
ros2 topic pub /cmd_vel_joy geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Links draaien
ros2 topic pub /cmd_vel_joy geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stoppen
ros2 topic pub /cmd_vel_joy geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 📊 Monitoring

Controleer of alles werkt:

```bash
# Alle topics bekijken
ros2 topic list

# LiDAR data bekijken
ros2 topic echo /scan

# Robot positie bekijken
ros2 topic echo /odom

# SLAM map bekijken
ros2 topic echo /map
```

## 🔧 Troubleshooting

### Service start niet
```bash
# Logs bekijken
sudo journalctl -u robot-auto -f

# Service opnieuw starten
sudo systemctl restart robot-auto
```

### Handmatige start voor debugging
```bash
cd /sbx/ros2
./start_robot.sh
```

### Alle ROS2 processen stoppen
```bash
pkill -f "ros2"
```

## 📱 Remote Monitoring

Voor remote monitoring via RViz:

1. Zorg dat je computer en Raspberry Pi op hetzelfde netwerk zijn
2. Stel ROS_DOMAIN_ID in op beide systemen
3. Start RViz op je computer: `ros2 run rviz2 rviz2`
4. Voeg de volgende displays toe:
   - **Fixed Frame**: `map`
   - **RobotModel**: Topic `/robot_description`
   - **LaserScan**: Topic `/scan`
   - **Map**: Topic `/map`
   - **TF**: Voor transform frames

## ✅ Wat er wordt gestart

Het startup script start automatisch:

1. **Robot State Publisher** - Publiceert robot transforms
2. **Controller Manager** - Beheert motor controllers
3. **LiDAR Node** - RPLiDAR A1/A2 scanner
4. **SLAM Toolbox** - Mapping en localization
5. **Joystick Control** - Voor handmatige besturing
6. **Twist Mux** - Prioriteert verschillende commando bronnen

## 🔄 Na een Herstart

Na een herstart van de Raspberry Pi:

1. De service start automatisch
2. Alle componenten worden gecontroleerd
3. Status wordt getoond in de logs
4. Robot is klaar voor gebruik

## 📞 Support

Als er problemen zijn:
1. Bekijk de logs: `sudo journalctl -u robot-auto -f`
2. Test handmatig: `./start_robot.sh`
3. Controleer hardware verbindingen (LiDAR, Arduino)
4. Verifieer USB poorten: `ls /dev/ttyUSB*` 