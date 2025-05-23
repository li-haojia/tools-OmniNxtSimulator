import argparse
import json
import os
import yaml
import carb
import numpy as np


# Default config for environment generation
default_config = {
    "launch_config": {
        "renderer": "RayTracedLighting",
        "headless": True,
        "livestream": 2,
    },
    "env_spacing": 20.0,
    "map_generator_config": {
        "num_obstacles": 200,
        "min_distance": 0.8,
        "obstacle_size_range": [0.3, 0.8],
        "obstacle_height_range": [0.5, 3.0],
        "num_floaters": 100,
        "floaters_size_range": [0.1, 0.5],
        "floaters_height_range": [1.0, 3.0]
    },
    "voxel_width": 0.1,
    "data_path": "data_apps/assets/random",
    "env_name": "generated_environment.usd",
    "pcd_name": "generated_environment.pcd",
    "close_app_after_run": True,
}
parser = argparse.ArgumentParser(description="Generate environment for trajectory planning")
parser.add_argument("--config", required=False, help="Path to config file (json or yaml)")
args = parser.parse_args()

config = default_config.copy()
if args.config and os.path.isfile(args.config):
    with open(args.config, "r") as f:
        if args.config.endswith(".json"):
            config.update(json.load(f))
        elif args.config.endswith(".yaml"):
            config.update(yaml.safe_load(f))

# Late import of runtime modules (the SimulationApp needs to be created before loading the modules)
from isaaclab.app import AppLauncher
# Create the simulation app
app_launcher = AppLauncher(config["launch_config"])
simulation_app = app_launcher.app

from omni.physx.scripts import physicsUtils
from isaacsim.core.utils import prims
from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaaclab.sim import SimulationContext
import isaaclab.sim as sim_utils
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from data_apps.synthetic_data.map_generator import MapGenerator
from pxr import UsdGeom, Gf, Usd


def save_as_pcd(points, filepath):
    """Save point cloud data to a PCD file."""
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    
    with open(filepath, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")
    
    print(f"Point cloud saved to {filepath}")

class SceneConfig:
    def __init__(self, env_spacing, num_envs=1, voxel_width=0.1):
        self.env_spacing = env_spacing
        self.num_envs = num_envs
        self.voxel_width = voxel_width

def setup_environment(config):
    """Setup the environment - either load from USD or generate using MapGenerator"""
    print(f"Generating environment using MapGenerator")
    stage = get_current_stage()
    
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")

    if not prims.is_prim_path_valid("/World"):
        prims.create_prim("/World", "Xform")
    
    scene_config = SceneConfig(env_spacing=config["env_spacing"], num_envs=1, voxel_width=config["voxel_width"])
    
    physics_sim = SimulationContext(sim_utils.SimulationCfg(dt=0.1))
    map_generator = MapGenerator(physics_sim, device="cuda")
    
    map_config = config["map_generator_config"]
    env_data = map_generator.create_environment(
        scene_config,
        num_obstacles=map_config["num_obstacles"],
        min_distance=map_config["min_distance"],
        obstacle_size_range=tuple(map_config["obstacle_size_range"]),
        obstacle_height_range=tuple(map_config["obstacle_height_range"]),
        num_floaters=map_config["num_floaters"],
        floaters_size_range=tuple(map_config["floaters_size_range"]),
        floaters_height_range=tuple(map_config["floaters_height_range"])
    )

    points = env_data["points"]
    pcd_path = os.path.join(config["data_path"], config["pcd_name"])
    save_as_pcd(points, pcd_path)
    print(f"Generated environment point cloud saved to {pcd_path}")
    
    return stage, points
        
 
def save_stage_as_usd(stage, output_path):
    """Save the current stage as USD file"""
    print(f"Saving stage to {output_path}")
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    stage.GetRootLayer().Export(output_path)
    print(f"Stage saved successfully to {output_path}")

def main():
    # Generate environment
    stage, points = setup_environment(config)
    
    # Save USD file
    usd_output_path = os.path.join(config["data_path"], config["env_name"])
    save_stage_as_usd(stage, usd_output_path)
    
    print(f"Environment generation completed:")
    print(f"- USD file: {usd_output_path}")

    # Ensure proper shutdown
    if config.get("close_app_after_run", True):
        if config["launch_config"]["headless"]:
            # simulation_app.close()
            sys.exit(0)
        else:
            # For non-headless mode, we need to handle the window properly
            while simulation_app.is_running():
                simulation_app.update()
                if not simulation_app.is_running():
                    break
            # simulation_app.close()
            sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, closing application...")
        # simulation_app.close()
        sys.exit(0)
    except Exception as e:
        print(f"Error occurred: {e}")
        # simulation_app.close()
        sys.exit(1) 