import argparse
import logging

from omni.isaac.lab.app import AppLauncher
def parse_arguments():
    """Parse command-line arguments.

    Returns:
        argparse.Namespace: Parsed arguments.
    """
    parser = argparse.ArgumentParser(description="Simulator for UAV with ROS2")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    return args

app_launcher = AppLauncher(parse_arguments())
simulation_app = app_launcher.app

# Enable ROS2 bridge extension
from omni.isaac.core.utils import extensions
extensions.enable_extension("omni.isaac.ros2_bridge")

# Configure logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


# Start your own code here
from simulator.uav_simulator import UAVSimulation
from vehicles.omninxt import OmniNxt

def main():
    """Main entry point for the simulation."""
    quadrotors = [
        OmniNxt(id=0, init_pos=(0.0, 0.5, 0.8), enable_cameras=True)
    ]
    simulation = UAVSimulation(simulation_app, quadrotors)
    simulation.run()


if __name__ == "__main__":
    main()