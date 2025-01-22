import argparse
import asyncio
import json
import math
import time
import os
import random

import yaml
from isaacsim import SimulationApp

# Default config (will be updated/extended by any other passed config arguments)
config = {
    "launch_config": {
        "renderer": "RayTracedLighting",
        "headless": True,
    },
    "trajectory_path": "/workspace/isaaclab/user_apps/assets/MFG_Factory_Welding_Demo/trajectory_100.hdf5",
    "sample_interval": 50,
    "start_group": 0,
    "end_group": -1,
    "robot": {
        "url": "/workspace/isaaclab/user_apps/data_apps/assets/OmniNxt_sdg_color.usd",
        "mass": 0.85,
        "radius": 0.15,
              "cameras": [{
                "name": "left_front_camera",
                "position": [0.0735, 0.0735, 0.067],
                "quaternion": [ 0.383, 0, 0, -0.924], #[ 0.924, 0, 0, 0.383],
                "resolution": [1280, 720],
                "camera_model": "fisheye",
                "fov": 200.0,
            }, {
                "name": "right_front_camera",
                "position": [0.0735, -0.0735, 0.067],
                "quaternion": [ 0.383, 0, 0, 0.924],#[  0.924, 0, 0, -0.383],
                "resolution": [1280, 720],
                "camera_model": "fisheye",
                "fov": 200.0,
            }, {
                "name": "right_back_camera",
                "position": [-0.0735, -0.0735, 0.067],
                "quaternion":  [ 0.924, 0, 0, 0.383], #[ 0.383, 0, 0, -0.924],
                "resolution": [1280, 720],
                "camera_model": "fisheye",
                "fov": 200.0,
            }, {
                "name": "left_back_camera",
                "position": [-0.0735, 0.0735, 0.067],
                "quaternion": [  0.924, 0, 0, -0.383], #[ 0.383, 0, 0, 0.924],
                "resolution": [1280, 720],
                "camera_model": "fisheye",
                "fov": 200.0,
            }, {
                "name": "driving_camera",
                "position": [0.12, 0.0, 0.067],
                "quaternion": [ 0.0, 0.0, 0.0, 1.0],
                "camera_model": "pinhole",
                "resolution": [640, 640],
                "focal_length": 15.0,
            },
        ],
    },
    "rt_subframes": 8,
    "env_url": "file:///workspace/isaaclab/user_apps/assets/MFG_Factory_Welding_Demo/MFG/MFG_Factory_Welding_Demo/MFG_Factory_Welding_Demo.usd",
    "writer": "BasicWriter",
    "writer_config": {
        "output_dir": "/workspace/isaaclab/user_apps/assets/MFG_Factory_Welding_Demo/path_tracking_sdg",
        "camera_params": True,
        "rgb": True,
        "distance_to_camera": True,
        "colorize_depth": True,
        "instance_segmentation": False,
        "colorize_instance_segmentation": False,
        "semantic_segmentation": False,
        "colorize_semantic_segmentation": False,
        "bounding_box_2d_tight": True,
        "bounding_box_3d": True,
    },
    "save_trajectory_info": True,
    "clear_previous_semantics": False,
    "close_app_after_run": True,
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

import data_apps.synthetic_data.scene_based_sdg_utils as scene_based_sdg_utils
from data_apps.synthetic_data.path_tracking_sdg_utils import TrajectoryDatabase
from omni.physx.scripts import physicsUtils
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles, quat_to_rot_matrix
from omni.isaac.core.utils.stage import get_current_stage, open_stage, print_stage_prim_paths, add_reference_to_stage
from omni.isaac.core.utils.semantics import count_semantics_in_scene
from omni.isaac.nucleus import get_assets_root_path
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import UsdGeom, Gf, UsdPhysics, Usd, UsdLux

def carb_settings():
    # Detailed settings for the replicator
    # https://docs.omniverse.nvidia.com/py/replicator/1.11.16/source/extensions/omni.replicator.core/docs/API.html#omni.replicator.core.settings.carb_settings
    # IO threads and queue size for writing data
    WRITE_THREADS = 64
    QUEUE_SIZE = 1000
    rep.settings.carb_settings("/omni/replicator/backend/writeThreads", WRITE_THREADS)
    rep.settings.carb_settings("/omni/replicator/backend/queueSize", QUEUE_SIZE)

    # Ray tracing motion blur settings
    rep.settings.carb_settings("/omni/replicator/captureOnPlay", False)
    rep.settings.carb_settings("/omni/replicator/captureMotionBlur", True)
    rep.settings.carb_settings("/rtx/rendermode", "RayTracedLighting")
    rep.settings.carb_settings("/rtx/post/aa/op", 3)
    rep.settings.carb_settings("/rtx/post/motionblur/maxBlurDiameterFraction", 0.02)
    rep.settings.carb_settings("/rtx/post/motionblur/exposureFraction", 1.0)
    rep.settings.carb_settings("/rtx/post/motionblur/numSamples", 8)

    # Light settings
    rep.settings.carb_settings("/rtx/shadows/enabled", True)
    rep.settings.carb_settings("/rtx/directLighting/domeLight/enabled", True)
    rep.settings.carb_settings("/rtx/directLighting/domeLight/enabledInReflections", True)

carb_settings()
if ":/" not in config["env_url"]:
    config["env_url"] = get_assets_root_path() + config["env_url"]
elif "file://" in config["env_url"]:
    usd_dir = os.path.dirname(config["env_url"].split("file://")[-1])
    os.chdir(usd_dir) # Avoid issues with relative paths in the USD file

usd_path = config["env_url"]
prim_path = f"/Env/{usd_path.split('/')[-1].split('.')[0]}"
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

stage = get_current_stage()
root_prim = stage.GetPrimAtPath(prim_path)

# Process lights in the stage
scene_based_sdg_utils.process_lights(stage, UsdLux.SphereLight, "sphere")
scene_based_sdg_utils.process_lights(stage, UsdLux.DiskLight, "sphere")

# Load sky
sky_usd_path = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Environments/2023_1/DomeLights/Dynamic/CumulusLight.usd"
rep.create.from_usd(usd=sky_usd_path)
sky = rep.get.prim_at_path(path="/Replicator/Ref_Xform/Ref/Looks/SkyMaterial/Shader")

# Disable capture on play (data generation will be triggered manually)
rep.orchestrator.set_capture_on_play(False)

# Load the light prims and make them visible
scene_based_sdg_utils.register_randomize_sun_light(sky)

# Clear any previous semantic data in the loaded stage
if config["clear_previous_semantics"]:
    stage = get_current_stage()
    scene_based_sdg_utils.remove_previous_semantics(stage)

word_prim = prims.create_prim("/World_custom")
cameras_xform = rep.create.xform(
    position=[0.0, 0.0, 0.0],
    rotation=[0.0, 0.0, 0.0],
    name="cameras",
    parent="/World_custom"
)
cameras_xform_path = '/World_custom/cameras'
cameras_xform_prim = stage.GetPrimAtPath(cameras_xform_path)

# Create cameras and render products
camera_prims = []
render_products = []
for camera_config in config["robot"]["cameras"]:
    rotation_euler = quat_to_euler_angles(camera_config["quaternion"]) * (180 / math.pi)
    projection_type = (
        "fisheye_polynomial" if camera_config["camera_model"] == "fisheye" else "pinhole"
    )
    camera_kwargs = {
        "position": camera_config["position"],
        "rotation": rotation_euler,
        "projection_type": projection_type,
        "name": camera_config["name"],
        "parent": "/World_custom/cameras",
    }
    if projection_type == "fisheye_polynomial":
        camera_kwargs["fisheye_max_fov"] = camera_config["fov"]
    else:
        camera_kwargs["focal_length"] = camera_config["focal_length"]
    camera_prim = rep.create.camera(**camera_kwargs)
    render_product = rep.create.render_product(
        camera_prim,
        resolution=camera_config["resolution"],
        name=camera_config["name"],
    )
    render_product.hydra_texture.set_updates_enabled(False)
    camera_prims.append(camera_prim)
    render_products.append(render_product)

# Make sure the writer type is in the registry
writer_type = config.get("writer", "BasicWriter")
if writer_type not in rep.WriterRegistry.get_writers():
    carb.log_error(f"Writer type {writer_type} not found in the registry, closing application..")
    simulation_app.close()

# Increase subframes if materials are not loaded on time, or ghosting artifacts appear on moving objects,
# see: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/subframes_examples.html
rt_subframes = config.get("rt_subframes", -1)

# Setup the randomizations to be triggered every frame
with rep.trigger.on_frame(interval=10):
    rep.randomizer.randomize_sun_light()

# Start the SDG
# Enable the render products for SDG
for rp in render_products:
    rp.hydra_texture.set_updates_enabled(True)

for _ in range(50):
    rep.orchestrator.step(delta_time=0.1, rt_subframes=rt_subframes)

env_semantics = count_semantics_in_scene("/Root")
print(f"[scene_based_sdg] Number of semantics in the environment: {env_semantics}")
db = TrajectoryDatabase(config["trajectory_path"])
def process_group(group_num, db, config, writer_type, render_products, cameras_xform, rt_subframes):
    try:
        # Setup the writer for the current group
        writer = rep.WriterRegistry.get(writer_type)
        writer_kwargs = config["writer_config"].copy()
        writer_kwargs["output_dir"] = os.path.join(writer_kwargs["output_dir"], f"group_{group_num}")
        print(f"[scene_based_sdg] Initializing {writer_type} with: {writer_kwargs}")
        writer.initialize(**writer_kwargs)
        writer.attach(render_products)

        # Load the trajectory for the current group
        trajectory = db.getTrajectory(group_num)
        interval = config.get("sample_interval", 100)
        used_samples = []
        for sample_counter in range(len(trajectory)):
            if sample_counter % interval != 0:
                continue
            used_samples.append(sample_counter)
            timestamp = trajectory.getTimestamp()[sample_counter]
            position = trajectory.getPosbyIndex(sample_counter)  # numpy array
            orientation = trajectory.getQuaternionbyIndex(sample_counter)  # numpy array [w, x, y, z]
            rotation = quat_to_euler_angles(orientation) * 180 / math.pi
            velocity = trajectory.getVelbyIndex(sample_counter)  # numpy array
            omega = trajectory.getOmgbyIndex(sample_counter) * 180 / math.pi  # numpy array

            xform_api = UsdGeom.Xformable(cameras_xform_prim)
            xform_api.ClearXformOpOrder()
            xform_api.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2]))
            xform_api.AddRotateXYZOp().Set(Gf.Vec3d(rotation[0], rotation[1], rotation[2]))

            physics_api = UsdPhysics.RigidBodyAPI.Apply(cameras_xform_prim)
            physics_api.CreateVelocityAttr().Set(Gf.Vec3f(float(velocity[0]), float(velocity[1]), float(velocity[2])))
            physics_api.CreateAngularVelocityAttr().Set(Gf.Vec3f(float(omega[0]), float(omega[1]), float(omega[2])))

            # Update the physics state
            next_timestamp = trajectory.getTimestamp()[sample_counter + 1] if sample_counter + 1 < len(trajectory) else timestamp
            dt = next_timestamp - timestamp
            start_time = time.time()
            rep.orchestrator.step(delta_time=dt, rt_subframes=rt_subframes)
            print(f"[scene_based_sdg] Group: {group_num}, Timestamp: {timestamp:.2f}s, Processing time: {time.time() - start_time:.2f}s")

        # Save the trajectory information
        if config.get("save_trajectory_info", True):
            trajectory_info = {
                "trajectory_path": config["trajectory_path"],
                "group_num": group_num,
                "used_samples": used_samples,
            }
        with open(os.path.join(writer_kwargs["output_dir"], "trajectory_info.json"), "w") as f:
            json.dump(trajectory_info, f)
        # Wait for the data to be written to disk
        rep.orchestrator.wait_until_complete()
        # Cleanup the writer
        writer.detach()
    except Exception as e:
        print(f"[scene_based_sdg] Error in group {group_num}: {e}")

def main():
    # Process each group in a separate process
    start_group = config.get("start_group", 0)
    end_group = config.get("end_group", -1)
    if end_group == -1:
        end_group = len(db)
    for group_num in range(start_group, end_group):
        process_group(group_num, db, config, writer_type, render_products, cameras_xform, rt_subframes)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[scene_based_sdg] Error: {e}")
    finally:
        # Cleanup render products
        for rp in render_products:
            rp.destroy()

        # Check if the application should keep running after the data generation (debug purposes)
        close_app_after_run = config.get("close_app_after_run", True)
        if config["launch_config"]["headless"]:
            if not close_app_after_run:
                print(
                    "[scene_based_sdg] 'close_app_after_run' is ignored when running headless. The application will be closed."
                )
        elif not close_app_after_run:
            print("[scene_based_sdg] The application will not be closed after the run. Make sure to close it manually.")
            while simulation_app.is_running():
                simulation_app.update()
        simulation_app.close()
