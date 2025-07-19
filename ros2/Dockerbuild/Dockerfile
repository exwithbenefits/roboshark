FROM ros:humble-ros-core

# Enable Universe repo and install repo management tools\ RUN apt-get update \
    && apt-get install -y software-properties-common apt-transport-https curl gnupg lsb-release \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# Set ROS 2 DDS and domain environment defaults
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=0

# Install core utilities, ROS 2 packages, Cyclone DDS, and system dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       openssh-server \
       supervisor \
       dos2unix \
       git \
       nano \
       net-tools \
       build-essential \
       python3-colcon-common-extensions \
       python3-pip \
       python3-serial \
       python3-rosdep \
       libserial-dev \
       fastdds-tools \
       libddsc0 \
       libcycloneddsidl0 \
       ros-humble-robot-state-publisher \
       ros-humble-joint-state-publisher \
       ros-humble-teleop-twist-keyboard \
       ros-humble-rplidar-ros \
       ros-humble-twist-mux \
       ros-humble-xacro \
       ros-humble-v4l2-camera \
       ros-humble-joy \
       ros-humble-teleop-twist-joy \
       ros-humble-navigation2 \
       ros-humble-nav2-bringup \
       ros-humble-ros2-control \
       ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Install additional peripherals, camera, and sensor libraries
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       i2c-tools \
       python3-rpi.gpio \
       python3-lgpio \
       v4l-utils \
       fswebcam \
       libcamera-apps \
       gpiod \
       python3-smbus \
       python3-libgpiod \
       ros-humble-rclpy \
       ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Configure SSH for root login + password authentication
RUN mkdir -p /var/run/sshd && \
    echo 'root:root' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Overwrite root entry in /etc/shadow with provided hash
RUN sed -i 's|^root:.*|root:$y$j9T$DHBTt9OvvV/7jikvwp.C.1$DL7xhMXFGCPbYkB2mfC.s4HV4oLVbrdFepijWf7XAM8:20204:0:99999:7:::|' /etc/shadow

# Append ROS 2 workspace sourcing and DDS env-vars to root's bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /sbx/ros2/install/setup.bash ]; then' >> /root/.bashrc && \
    echo '  source /sbx/ros2/install/setup.bash' >> /root/.bashrc && \
    echo 'fi' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc && \
    echo 'export ROS_DISCOVERY_SERVER=127.0.0.1:11811' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc && \
    echo 'export ROS_SUPER_CLIENT=1' >> /root/.bashrc

# Initialize rosdep and prepare workspace
RUN rosdep init && rosdep update
RUN mkdir -p /sbx/ros2/src
WORKDIR /sbx/ros2

# Build the ROS 2 workspace
RUN /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

# Supervisor and entrypoint
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh

# Expose SSH, HTTP/S, and DDS discovery ports
EXPOSE 60022 80 443 11811 11812
EXPOSE 7400-7700/udp

ENTRYPOINT ["/usr/bin/supervisord","-n","-c","/etc/supervisor/supervisord.conf"]
