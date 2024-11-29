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
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
extensions.enable_extension("omni.isaac.ros2_bridge")

# Configure logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


# Start your own code here
from simulator.uav_simulator import UAVSimulation
from vehicles.omninxt import OmniNxt

def generate_quadrotors(num_robots):
    quadrotors = []
    spacing = 1.0
    grid_size = int(num_robots**0.5)

    # Calculate offset to center the grid around (0, 0)
    offset = (grid_size - 1) * spacing / 2

    for i in range(num_robots):
        row = i // grid_size
        col = i % grid_size
        x = col * spacing - offset
        y = row * spacing - offset
        quadrotors.append(OmniNxt(id=i, init_pos=(x, y, 0.5), enable_cameras=False))
    return quadrotors

def main():
    """Main entry point for the simulation."""
    env_usd_paths = [
        # f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
        f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd",
        # f"/workspace/isaaclab/user_apps/assets/Demos/AEC/TowerDemo/TowerDemopack/World_TowerDemopack.usd",
        # f"/workspace/isaaclab/user_apps/assets/Demos/AEC/TowerDemo/CityDemopack/World_CityDemopack.usd",
    ]
    simulation = UAVSimulation(simulation_app, env_usd_paths, generate_quadrotors(4))
    simulation.run()


if __name__ == "__main__":
    main()