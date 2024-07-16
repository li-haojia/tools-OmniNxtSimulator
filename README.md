# tools-OmniNxtSimulator

OmniNxt simulator based on Isaac Sim

## Installation

1. Install Isaac Sim

2. Install ROS2 humble

3. Clone this repository

```bash
git clone https://github.com/D2SLAM-Fusion/tools-OmniNxtSimulator.git
cd tools-OmniNxtSimulator
./scripts/init.sh
```

3. Setup the environment

[PegasusSimulator](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)

4. ROS2 <-> ROS1 communication bridge

This bridge will automatically map the ROS2 topics to ROS1 topics. This is useful for the ROS1 packages that are not yet ported to ROS2.

```bash
cd ros_bridge
docker-compose up -d
```