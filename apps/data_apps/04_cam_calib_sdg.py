# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""Generate offline synthetic dataset
"""

import argparse
import json
import math
import os
import random
import numpy as np
import yaml
from isaacsim import SimulationApp

# Default config (will be updated/extended by any other passed config arguments)
config = {
    "launch_config": {
        "renderer": "RayTracedLighting",
        "headless": True,
    },
    "camera": {
        "name": "CalibrationCamera",
        "position": [0.0, 0.0, 0.0],
        "quaternion": [ 0, 0, 0, 1],
        "resolution": [1280, 720],
        "projection_type": "fisheye_polynomial",
        "fov": 200.0,
    },
    "rt_subframes": 16,
    "num_frames": 2000,
    "env_url": "/Isaac/Environments/Grid/default_environment.usd",
    "writer": "BasicWriter",
    "writer_config": {
        "output_dir": "/workspace/isaaclab/user_apps/data_apps/output/cam_calib_sdg",
        "rgb": True,
        "distance_to_camera": False,
    },
    "april_grid": {
        "url": "file:///workspace/isaaclab/user_apps/data_apps/assets/Aprilgrid_6x6_0.3_0.1234567/Aprilgrid_6x6_0_3_0_1234567.usd",
        "class": "april_grid",
    },
}

import carb

# Check if there are any config files (yaml or json) are passed as arguments
parser = argparse.ArgumentParser()
parser.add_argument("--config", required=False, help="Include specific config parameters (json or yaml))")
args, unknown = parser.parse_known_args()
args_config = {}
if args.config and os.path.isfile(args.config):
    print("File exist")
    with open(args.config, "r") as f:
        if args.config.endswith(".json"):
            args_config = json.load(f)
        elif args.config.endswith(".yaml"):
            args_config = yaml.safe_load(f)
        else:
            carb.log_warn(f"File {args.config} is not json or yaml, will use default config")
else:
    carb.log_warn(f"File {args.config} does not exist, will use default config")

# If there are specific writer parameters in the input config file make sure they are not mixed with the default ones
if "writer_config" in args_config:
    config["writer_config"].clear()

# Update the default config dictionay with any new parameters or values from the config file
config.update(args_config)

# Create the simulation app with the given launch_config
simulation_app = SimulationApp(launch_config=config["launch_config"])

# Late import of runtime modules (the SimulationApp needs to be created before loading the modules)
import omni.replicator.core as rep
import omni.usd

# Custom util functions for the example
import data_apps.synthetic_data.scene_based_sdg_utils as scene_based_sdg_utils
from isaacsim.core.utils import prims
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.utils.stage import get_current_stage, open_stage, add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from pxr import Gf, Usd, UsdGeom
import omni.kit.actions.core

def get_pose_distribution(april_grid_pos_gf, num_frames):
    pose_list = []
    # Down side of the april grid
    up_frames = num_frames // 2
    calibration_cam_min_down = np.array([april_grid_pos_gf[0] - 1.2, april_grid_pos_gf[1] - 1.2, april_grid_pos_gf[2] - 1.2])
    calibration_cam_max_down = np.array([april_grid_pos_gf[0] + 1.2, april_grid_pos_gf[1] + 1.2, april_grid_pos_gf[2] - 0.4])
    for i in range(up_frames):
        pose = np.random.uniform(calibration_cam_min_down, calibration_cam_max_down)
        pose_list.append(pose.tolist())

    # Up side of the april grid
    calibration_cam_min_up = np.array([april_grid_pos_gf[0] - 1.2, april_grid_pos_gf[1] - 1.2, april_grid_pos_gf[2] + 0.4])
    calibration_cam_max_up = np.array([april_grid_pos_gf[0] + 1.2, april_grid_pos_gf[1] + 1.2, april_grid_pos_gf[2] + 1.2])
    for i in range(num_frames - up_frames):
        pose = np.random.uniform(calibration_cam_min_up, calibration_cam_max_up)
        pose_list.append(pose.tolist())

    return pose_list

def carb_settings():
    # Detailed settings for the replicator
    # https://docs.omniverse.nvidia.com/py/replicator/1.11.16/source/extensions/omni.replicator.core/docs/API.html#omni.replicator.core.settings.carb_settings
    # IO threads and queue size for writing data
    WRITE_THREADS = 64
    QUEUE_SIZE = 1000
    rep.settings.carb_settings("/omni/replicator/backend/writeThreads", WRITE_THREADS)
    rep.settings.carb_settings("/omni/replicator/backend/queueSize", QUEUE_SIZE)
    
    # switches to camera lighting
    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    action.execute()
        

carb_settings()
# Open the given environment in a new stage
if ":/" not in config["env_url"]:
    config["env_url"] = get_assets_root_path() + config["env_url"]
elif "file://" in config["env_url"]:
    usd_dir = os.path.dirname(config["env_url"].split("file://")[-1])
    os.chdir(usd_dir) # Avoid issues with relative paths in the USD file

usd_path = config["env_url"]
prim_path = f"/World/{usd_path.split('/')[-1].split('.')[0]}"
print(f"Loading stage at {prim_path}")
add_reference_to_stage(usd_path, prim_path)
stage = Usd.Stage.Open(usd_path)
unit = UsdGeom.GetStageMetersPerUnit(stage)
print(f"Meters per unit: {unit}")

if unit != 1.0:
    # Need to scale the stage to match the 1 unit = 1 meter
    try:
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(root_prim)
        xform.ClearXformOpOrder()
        xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(unit, unit, unit))
    except Exception as e:
        print(f"Error scaling stage: {e}")
        exit(1)


# Spawn a new prim for the AprilGrid
april_grid_prim = prims.create_prim(
    prim_path="/World/april_grid",
    position=(0, 0, 2),
    scale=(0.01, 0.01, 0.01), # Scale the grid to 1unit = 1m
    orientation=(0, 0, 0, 1),
    usd_path=config["april_grid"]["url"],
)
april_grid = rep.get.prim_at_path("/World/april_grid")
april_grid_tf = omni.usd.get_world_transform_matrix(april_grid_prim)
april_grid_pos_gf = april_grid_tf.ExtractTranslation()

# Spawn a camera which will be used for calibration
camera_config = config["camera"]
rotation_euler = quat_to_euler_angles(camera_config["quaternion"]) * (180 / math.pi)
camera_kwargs = {
    "position": camera_config["position"],
    "rotation": rotation_euler,
    "projection_type": camera_config["projection_type"],
    "name": camera_config["name"],
    "clipping_range": (0.1, 1000.0),
    "parent": "/World",
}
if camera_config["projection_type"] == "fisheye_polynomial":
    camera_kwargs["fisheye_max_fov"] = camera_config["fov"]
else:
    camera_kwargs["focal_length"] = camera_config["focal_length"]
calibration_cam = rep.create.camera(**camera_kwargs)

# Create render products for the custom cameras and attach them to the writer
resolution = camera_config.get("resolution", [1280, 720])
april_grid_rp = rep.create.render_product(calibration_cam, resolution, name="AprilGrid")
april_grid_rp.hydra_texture.set_updates_enabled(False)

# If output directory is relative, set it relative to the current working directory
if not os.path.isabs(config["writer_config"]["output_dir"]):
    config["writer_config"]["output_dir"] = os.path.join(os.getcwd(), config["writer_config"]["output_dir"])
print(f"[scene_based_sdg] Output directory={config['writer_config']['output_dir']}")

# Make sure the writer type is in the registry
writer_type = config.get("writer", "BasicWriter")
if writer_type not in rep.WriterRegistry.get_writers():
    carb.log_error(f"Writer type {writer_type} not found in the registry, closing application..")
    simulation_app.close()

# Get the writer from the registry and initialize it with the given config parameters
writer = rep.WriterRegistry.get(writer_type)
writer_kwargs = config["writer_config"]
print(f"[scene_based_sdg] Initializing {writer_type} with: {writer_kwargs}")
writer.initialize(**writer_kwargs)

# Attach writer to the render products
writer.attach([april_grid_rp])
    
# Setup the randomizations to be triggered every frame
camera_poses = get_pose_distribution(april_grid_pos_gf, config["num_frames"])
with rep.trigger.on_frame():
    with calibration_cam:
        rep.modify.pose(
            position=rep.distribution.sequence(camera_poses),
            look_at=str(april_grid_prim.GetPrimPath()),
        )
    with april_grid:
        rep.modify.pose(
            rotation=rep.distribution.choice([(0, 0, 0), (0, 0, 90), (0, 0, -90), (0, 0, 180)]),
        )

# Increase subframes if materials are not loaded on time, or ghosting artifacts appear on moving objects,
# see: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/subframes_examples.html
rt_subframes = config.get("rt_subframes", -1)

# Enable the render products for SDG
april_grid_rp.hydra_texture.set_updates_enabled(True)

# Start the SDG
num_frames = config.get("num_frames", 0)
for i in range(num_frames):
    print(f"[scene_based_sdg] \t Capturing frame {i}")
    # Trigger any on_frame registered randomizers and the writers (delta_time=0.0 to avoid advancing the timeline)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=rt_subframes)

# Wait for the data to be written to disk
rep.orchestrator.wait_until_complete()

# Cleanup writer and render products
writer.detach()
april_grid_rp.destroy()
simulation_app.close()
