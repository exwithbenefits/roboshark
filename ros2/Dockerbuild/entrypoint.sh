#!/usr/bin/env bash
set -e

# Update SSH to listen on port 60022
sed -i 's/#Port 22/Port 60022/' /etc/ssh/sshd_config

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
if [ -f /sbx/ros2/install/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  source /sbx/ros2/install/setup.bash
fi

export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Run precheck script if it exists\ nif [ -f /sbx/ros2/startup/precheck.sh ]; then
  echo "Running precheck.sh..."
  bash /sbx/ros2/startup/precheck.sh
fi

exec "$@"
