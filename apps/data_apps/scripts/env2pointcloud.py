"""
This script generates a 3D occupancy map using NVIDIA Omniverse Isaac Sim and saves it as a PCD file.
Run the script using the following command:
./gui.sh env2_pointcloud.py
"""

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
# Import necessary libraries
import carb
import numpy as np
from pxr import UsdPhysics, Sdf, Gf, UsdGeom, Usd
from omni.isaac.occupancy_map.bindings import _occupancy_map
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.nucleus import get_assets_root_path
import os
import numpy as np
import textwrap

def save_point_cloud_to_pcd(points, filename="environment_point_cloud.pcd"):
    """
    Save point cloud as a .pcd file in ASCII format.

    Parameters:
    - points (array-like): An iterable of points, where each point is an iterable of three coordinates (x, y, z).
    - filename (str): The name of the file to save the point cloud to.
    """
    points = np.asarray(points, dtype=np.float32)
    print(f"Points shape: {points.shape}")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Points array must be of shape (N, 3)")

    num_points = points.shape[0]

    header = textwrap.dedent(f"""\
        # .PCD v0.7 - Point Cloud Data file format
        VERSION 0.7
        FIELDS x y z
        SIZE 4 4 4
        TYPE F F F
        COUNT 1 1 1
        WIDTH {num_points}
        HEIGHT 1
        VIEWPOINT 0 0 0 1 0 0 0
        POINTS {num_points}
        DATA ascii
        """)

    try:
        with open(filename, 'w', encoding='utf-8') as file:
            file.write(header)
            np.savetxt(file, points, fmt="%.6f %.6f %.6f")
        print(f"Point cloud saved to {filename}")
    except IOError as e:
        print(f"Failed to save point cloud to {filename}: {e}")

# function to generate and save occupancy map
def generate_and_save_occupancy_map(usd_paths, output_path,cell_size = 0.2, origin=(0.0,0.0,0.0), min_bound=(-2.0,-2.0,-0.5), max_bound=(2.0,2.0,2.0)):
    """
    Generate a 3D occupancy map using NVIDIA Omniverse Isaac Sim and save it as a PLY file.
    """
    # Get the assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    # Create a simulation context
    context = SimulationContext(sim_utils.SimulationCfg(dt=0.1))

    # Load the environment stage
    for usd_path in usd_paths:
        prim_path = f"/{usd_path.split('/')[-1].split('.')[0]}"
        print(f"Loading stage at {prim_path}")
        add_reference_to_stage(usd_path, prim_path)
        stage = Usd.Stage.Open(usd_path)
        unit = UsdGeom.GetStageMetersPerUnit(stage)
        print(f"Stage unit: {unit}")
        if unit != 1.0:
            # Need to scale the stage to match the 1 unit = 1 meter
            scale_vector = Gf.Vec3f(float(unit), float(unit), float(unit))
            stage = omni.usd.get_context().get_stage()
            root_prim = stage.GetPrimAtPath(prim_path)
            xform = UsdGeom.Xformable(root_prim)
            xform.AddScaleOp().Set(scale_vector)

    # Get the stage and define the physics scene
    # UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    # print("Physics scene defined")
    context_get = omni.usd.get_context()
    # Play the timeline to ensure physics is enabled
    physx = omni.physx.acquire_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    context.step()
    print("Timeline playing")
    occupancy_map = _occupancy_map.Generator(physx, stage_id)
    context.step()

    # Configure the occupancy map generator
    cell_size = 0.2
    occupancy_map.update_settings(cell_size, 1, 1, 0.5)
    occupancy_map.set_transform(origin, min_bound, max_bound)

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
    save_point_cloud_to_pcd(points, output_path)

# Run the async function
if __name__ == "__main__":
    paths = [
        # f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd",
        # f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd",
        f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    ]
    output_path = "occupancy_map.pcd"
    cell_size = 0.2
    origin = (0.0,0.0,0.0) # x y z
    min_bound = (-28.0, -25.0, -0.5) # x y z
    max_bound = (9.0, 32.0, 9.5) # x y z

    generate_and_save_occupancy_map(paths, output_path, cell_size, origin, min_bound, max_bound)