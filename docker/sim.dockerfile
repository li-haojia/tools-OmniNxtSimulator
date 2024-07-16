FROM nvcr.io/nvidia/isaac-sim:2023.1.1

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2
RUN apt update && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y ros-humble-desktop ros-dev-tools && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Install mavros
RUN apt update && \
    apt install -y ros-humble-mavros ros-humble-mavros-extras && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Install QGroundControl
RUN apt update && \
    apt-get remove modemmanager -y && \
    apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y && \
    apt install libfuse2 -y && \
    apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    cd /usr/bin && \
    wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && \
    chmod +x QGroundControl.AppImage

# PX4 Dependencies
RUN apt update && \
    apt install -y \
    git \
    make \
    cmake \
    python3-pip && \
    python3 -m pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*