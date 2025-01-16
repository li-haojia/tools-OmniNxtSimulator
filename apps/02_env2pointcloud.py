import omni
import argparse
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

from omni.isaac.core.utils import extensions
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.occupancy_map")
simulation_app.update()
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

import omni.isaac.lab.sim as sim_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
# Import necessary libraries
import carb
import numpy as np
from pxr import UsdPhysics, Sdf
from omni.isaac.occupancy_map.bindings import _occupancy_map
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.nucleus import get_assets_root_path
import os

from pxr import UsdGeom, Usd
import numpy as np
import open3d as o3d  # 用于保存点云

def save_point_cloud_to_pcd(points, filename="environment_point_cloud.pcd"):
    """
    Save point cloud as .pcd file
    """
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, point_cloud)
    print(f"Point cloud saved to {filename}")

# function to generate and save occupancy map
def generate_and_save_occupancy_map():
    """
    Generate a 3D occupancy map using NVIDIA Omniverse Isaac Sim and save it as a PLY file.
    """
    # Get the assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    # Load the Simple Room environment
    stage_path = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd" #os.path.join(assets_root_path, "Isaac/Environments/Simple_Room/simple_room.usd")
    stage = open_stage(stage_path)

    # Get the stage and define the physics scene
    # UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    # print("Physics scene defined")
    context = SimulationContext(sim_utils.SimulationCfg(dt=0.1))
    context_get = omni.usd.get_context()
    # Play the timeline to ensure physics is enabled
    physx = omni.physx.acquire_physx_interface()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    context.step()
    print("Timeline playing")
    occupancy_map = _occupancy_map.Generator(physx, context_get.get_stage_id())
    context.step()

    # Configure the occupancy map generator
    cell_size = 0.2
    # occupancy_map.set_cell_size(cell_size)
    occupancy_map.update_settings(cell_size, 1, 0, 0.5)
    occupancy_map.set_transform((0.0, 0.0, 0.0), (-28.0, -25.0, -0.5), (9.0, 32.0, 9.5))

    context.step()
    print("Occupancy map generator configured")

    # Generate the occupancy map
    occupancy_map.generate3d()
    context.step()
    timeline.stop()
    print("Occupancy map generated")

    # Retrieve the 3D points from the occupancy map
    points = occupancy_map.get_occupied_positions()

    print(f"Number of occupied points: {len(points)}")

    # Save the points to a PCD file
    output_file_path = "occupancy_map.pcd"  # Replace with your desired file path
    save_point_cloud_to_pcd(points, output_file_path)

# Run the async function
if __name__ == "__main__":
    generate_and_save_occupancy_map()