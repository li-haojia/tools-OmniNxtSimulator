"""
This script generates a 3D occupancy map using NVIDIA Omniverse Isaac Sim and saves it as a PCD file.
Run the script using the following command:
./stream.sh env2pointcloud.py
"""

import omni
import argparse
import asyncio
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

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.occupancy_map")
simulation_app.update()
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.occupancy_map.bindings import _occupancy_map
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.lab.sim import SimulationContext
from omni.kit.async_engine import run_coroutine
import omni.isaac.lab.sim as sim_utils
from pxr import Gf, UsdGeom, Usd
import numpy as np
import textwrap
import carb
import math
import os

MAX_CONCURRENT_TASKS = int(os.cpu_count())
semaphore = asyncio.Semaphore(MAX_CONCURRENT_TASKS)
context = SimulationContext(sim_utils.SimulationCfg(dt=0.1))

def write_points_to_file(points, filename):
    points = np.asarray(points, dtype=np.float32)
    print(f"Points shape: {points.shape}")
    if points.shape[0] == 0:
        return
    if points.shape[1] != 3:
        raise ValueError("Points should have 3 columns (x, y, z)")
    with open(filename, 'w', encoding='utf-8') as file:
        np.savetxt(file, points, fmt="%.6f %.6f %.6f")

def save_point_cloud_to_pcd(num_sub_boxes, filename="environment_point_cloud.pcd"):
    """
    Save point cloud as a .pcd file in ASCII format.

    Parameters:
    - points (array-like): An iterable of points, where each point is an iterable of three coordinates (x, y, z).
    - filename (str): The name of the file to save the point cloud to.
    """
    if os.path.exists(filename):
        os.remove(filename)

    num_points = 0
    for i in range(num_sub_boxes):
        if not os.path.exists(f"{filename}_{i}.txt"):
            continue
        with open(f"{filename}_{i}.txt", 'r') as file:
            lines = file.readlines()
            num_points += len(lines)
            with open(filename, 'a') as pcd_file:
                for line in lines:
                    pcd_file.write(line)
        # Remove the file after writing to pcd file
        os.remove(f"{filename}_{i}.txt")

    print(f"Total number of points: {num_points}")
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
    with open(filename, 'r') as file:
        lines = file.readlines()
        lines.insert(0, header)
    
    with open(filename, 'w') as file:
        file.writelines(lines)


def split_bounding_box(range_min, range_max, cell_size, max_size=100):
    """
    Split the bounding box into smaller boxes with a maximum size of max_size * cell_size.
    
    Parameters:
    - range_min (Gf.Vec3f): Minimum coordinates of the bounding box.
    - range_max (Gf.Vec3f): Maximum coordinates of the bounding box.
    - cell_size (float): Cell size for each point.
    - max_size (int): Maximum number of cells per axis in each sub-box.
    
    Returns:
    - List of tuples containing min and max coordinates for each sub-box.
    """
    sub_boxes = []
    steps = [
        math.ceil((range_max[i] - range_min[i]) / (cell_size * max_size)) 
        for i in range(3)
    ]
    steps = [max(1, step) for step in steps]  # Ensure at least one step per axis

    x_steps = steps[0]
    y_steps = steps[1]
    z_steps = steps[2]

    x_step_size = (range_max[0] - range_min[0]) / x_steps
    y_step_size = (range_max[1] - range_min[1]) / y_steps
    z_step_size = (range_max[2] - range_min[2]) / z_steps

    for i in range(x_steps):
        for j in range(y_steps):
            for k in range(z_steps):
                min_corner = (
                    range_min[0] + i * x_step_size,
                    range_min[1] + j * y_step_size,
                    range_min[2] + k * z_step_size
                )
                max_corner = (
                    min_corner[0] + x_step_size,
                    min_corner[1] + y_step_size,
                    min_corner[2] + z_step_size
                )
                sub_boxes.append((min_corner, max_corner))
    return sub_boxes

from asyncio import Semaphore

async def process_sub_box(id, min_range, max_range, cell_size, origin, output_path="environment_point_cloud.pcd"):
    async with semaphore:
        try:
            carb.log_info(f"Processing sub-box {id}")
            physx = omni.physx.acquire_physx_interface()
            stage_id = omni.usd.get_context().get_stage_id()
            timeline = omni.timeline.get_timeline_interface()
            await omni.kit.app.get_app().next_update_async()
            timeline.play()
            await omni.kit.app.get_app().next_update_async()
            occupancy_map = _occupancy_map.Generator(physx, stage_id)
            occupancy_map.update_settings(cell_size, 4, 5, 6)
            occupancy_map.set_transform(origin, min_range, max_range)
            carb.log_info(f"Generating occupancy map for sub-box {id}")
            occupancy_map.generate3d()
            points = occupancy_map.get_occupied_positions()
            print(f"Sub-box {id}: {min_range}, {max_range} | Points: {len(points)}")
            write_points_to_file(points, f"{output_path}_{id}.txt")
        except Exception as e:
            print(f"Error processing sub-box {id}: {e}")

# function to generate and save occupancy map
async def generate_and_save_occupancy_map(usd_paths, output_path, cell_size = 0.2, origin=(0.0,0.0,0.0)):
    """
    Generate a 3D occupancy map using NVIDIA Omniverse Isaac Sim and save it as a PLY file.
    """
    # Get the assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    # Load the environment stage
    for usd_path in usd_paths:
        prim_path = f"/Env/{usd_path.split('/')[-1].split('.')[0]}"
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

    # Get env bbox
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath("/Env")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
    bbox = bbox_cache.ComputeWorldBound(prim)
    range_min = bbox.GetRange().GetMin()
    range_max = bbox.GetRange().GetMax()
    print(f"Stage bounding box: {range_min}, {range_max}")
    sub_boxes = split_bounding_box(range_min, range_max, cell_size)
    print(f"Number of sub-boxes: {len(sub_boxes)}")

    tasks = []
    for i, (min_range, max_range) in enumerate(sub_boxes):
        task = run_coroutine(process_sub_box(i, min_range, max_range, cell_size, origin, output_path))   
        tasks.append(task)
        
    # Await all tasks to run them in parallel
    await asyncio.gather(*tasks)

    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    # Save the points to a PCD file
    save_point_cloud_to_pcd(len(sub_boxes), output_path)

async def main():
    paths = [
        # f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd",
        f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd",
        # f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
        # f"/workspace/isaaclab/user_apps/assets/Demos/AEC/TowerDemo/CityDemopack/World_CityDemopack.usd"
    ]
    output_path = "occupancy_map.pcd"
    cell_size = 0.2
    origin = (0.0, 0.0, 0.0)  # x y z

    print("Generating occupancy map...")
    await generate_and_save_occupancy_map(paths, output_path, cell_size, origin)
    print("Occupancy map generation complete")

# Run the async function
if __name__ == "__main__":
    task = asyncio.ensure_future(main())
    while not task.done():
        omni.kit.app.get_app().update()
        