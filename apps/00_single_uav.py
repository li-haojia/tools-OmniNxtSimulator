import argparse
import logging

from isaaclab.app import AppLauncher
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
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaacsim.core.utils import extensions
extensions.enable_extension("isaacsim.ros2.bridge")

# Configure logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


# Start your own code here
from simulator.uav_simulator import UAVSimulation
from vehicles.omninxt import OmniNxt

def main():
    """Main entry point for the simulation."""

    env_usd_paths = [
        f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
        # f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd",
        # f"/workspace/isaaclab/user_apps/assets/Demos/AEC/TowerDemo/TowerDemopack/World_TowerDemopack.usd",
        # f"/workspace/isaaclab/user_apps/assets/Demos/AEC/TowerDemo/CityDemopack/World_CityDemopack.usd",
    ]
    quadrotors = [
        OmniNxt(id=0, init_pos=(0.0, 0.0, 2), enable_cameras=True)
    ]
    simulation = UAVSimulation(simulation_app, env_usd_paths, quadrotors)
    simulation.run()


if __name__ == "__main__":
    main()