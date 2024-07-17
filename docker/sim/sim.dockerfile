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

# User setup
ARG USER=coder
RUN useradd --groups sudo --shell /bin/bash ${USER} --create-home \
	&& echo "${USER} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USER} \
	&& chmod 0440 /etc/sudoers.d/${USER}

# Desktop setup
RUN apt update && \
    apt install -y \
    iputils-ping \
    net-tools && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

USER ${USER}
# Install QGroundControl
RUN sudo usermod -a -G dialout ${USER} && \
    sudo apt update && \
    sudo apt-get remove modemmanager -y && \
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y && \
    sudo apt install libfuse2 -y && \
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y && \
    sudo apt clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    cd /home/${USER} && \
    wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && \
    chmod +x QGroundControl.AppImage && \
    ./QGroundControl.AppImage --appimage-extract && \
    mv squashfs-root QGroundControl && \
    echo "alias qgc='/home/${USER}/QGroundControl/QGroundControl'" >> ~/.bashrc

USER root