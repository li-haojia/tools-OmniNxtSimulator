# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import random

import numpy as np
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.bounds import compute_combined_aabb, compute_obb, create_bbox_cache, get_obb_corners
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.core.utils.semantics import remove_all_semantics
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# Process lights in the stage and create them in the replicator
def process_lights(stage, usd_light_class, create_type):
    """"
    Process lights in the stage and create them in the replicator.
    Args:
        stage: USD stage.
        usd_light_class: USD class for the light, either UsdLux.SphereLight or UsdLux.DiskLight.
        create_type: Replicator create type for the light, either "sphere" or "disk".
    """
    light_prims = [prim for prim in stage.Traverse() if prim.IsA(usd_light_class)]

    for prim in light_prims:
        # Get the light position and rotation
        transform = omni.usd.get_world_transform_matrix(prim)
        transform = transform.GetOrthonormalized(issueWarning=True)
        position = transform.ExtractTranslation()
        rotation = transform.ExtractRotation()
        euler_angles = rotation.Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))

        # Get the light intensity
        intensity_attr = prim.GetAttribute("intensity")
        intensity = intensity_attr.Get() if intensity_attr.IsValid() else 1.0

        # Create the light in the replicator
        rep.create.light(
            position=position,
            rotation=euler_angles,
            light_type=create_type,
            intensity=intensity
        )

# Add colliders to Gprim and Mesh descendants of the root prim
def add_colliders(root_prim, approx_type="convexHull"):
    # Iterate descendant prims (including root) and add colliders to mesh or primitive types
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            # Physics
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
        # Add mesh specific collision properties only to mesh types
        if desc_prim.IsA(UsdGeom.Mesh):
            # Add mesh collision properties to the mesh (e.g. collider aproximation type)
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set(approx_type)


# Clear any previous semantic data in the stage
def remove_previous_semantics(stage, recursive: bool = False):
    prims = stage.Traverse()
    for prim in prims:
        remove_all_semantics(prim, recursive)


# Run a simulation
def simulate_falling_objects(forklift_prim, assets_root_path, config, max_sim_steps=250, num_boxes=8):
    # Create the isaac sim world to run any physics simulations
    world = World(physics_dt=1.0 / 90.0, stage_units_in_meters=1.0)

    # Set a random relative offset to the pallet using the forklift transform as a base frame
    forklift_tf = omni.usd.get_world_transform_matrix(forklift_prim)
    pallet_offset_tf = Gf.Matrix4d().SetTranslate(Gf.Vec3d(random.uniform(-1, 1), random.uniform(-4, -3.6), 0))
    pallet_pos = (pallet_offset_tf * forklift_tf).ExtractTranslation()

    # Spawn a pallet prim at a random offset from the forklift
    pallet_prim = prims.create_prim(
        prim_path=f"/World/SimulatedPallet",
        position=pallet_pos,
        orientation=euler_angles_to_quat([0, 0, random.uniform(0, math.pi)]),
        usd_path=assets_root_path + config["pallet"]["url"],
        semantic_label=config["pallet"]["class"],
    )

    # Wrap the pallet as simulation ready with a simplified collider
    add_colliders(pallet_prim, approx_type="boundingCube")
    pallet_rigid_prim = RigidPrim(prim_path=str(pallet_prim.GetPrimPath()))
    pallet_rigid_prim.enable_rigid_body_physics()

    # Use the height of the pallet as a spawn base for the boxes
    bb_cache = create_bbox_cache()
    spawn_height = bb_cache.ComputeLocalBound(pallet_prim).GetRange().GetSize()[2] * 1.1

    # Keep track of the last box to stop the simulation early once it stops moving
    last_box = None
    # Spawn boxes falling on the pallet
    for i in range(num_boxes):
        # Spawn the carbox prim by creating a new Xform prim and adding the USD reference to it
        box_prim = prims.create_prim(
            prim_path=f"/World/SimulatedCardbox_{i}",
            position=pallet_pos + Gf.Vec3d(random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2), spawn_height),
            orientation=euler_angles_to_quat([0, 0, random.uniform(0, math.pi)]),
            usd_path=assets_root_path + config["cardbox"]["url"],
            semantic_label=config["cardbox"]["class"],
        )

        # Get the next spawn height for the box
        spawn_height += bb_cache.ComputeLocalBound(box_prim).GetRange().GetSize()[2] * 1.1

        # Wrap the prim as simulation ready with a simplified collider
        add_colliders(box_prim, approx_type="boundingCube")
        box_rigid_prim = RigidPrim(prim_path=str(box_prim.GetPrimPath()))
        box_rigid_prim.enable_rigid_body_physics()

        # Cache the rigid prim
        last_box = box_rigid_prim

    # Reset the world to handle the physics of the newly created rigid prims
    world.reset()

    # Simulate the world for the given number of steps or until the highest box stops moving
    for i in range(max_sim_steps):
        world.step(render=False)
        if last_box and np.linalg.norm(last_box.get_linear_velocity()) < 0.001:
            print(f"[scene_based_sdg] Simulation finished at step {i}..")
            break


# Register the boxes and materials randomizer graph
def register_scatter_boxes(pallet_prim, assets_root_path, config):
    # Calculate the bounds of the prim to create a scatter plane of its size
    bb_cache = create_bbox_cache()
    bbox3d_gf = bb_cache.ComputeLocalBound(pallet_prim)
    prim_tf_gf = omni.usd.get_world_transform_matrix(pallet_prim)

    # Calculate the bounds of the prim
    bbox3d_gf.Transform(prim_tf_gf)
    range_size = bbox3d_gf.GetRange().GetSize()

    # Get the quaterion of the prim in xyzw format from usd
    prim_quat_gf = prim_tf_gf.ExtractRotation().GetQuaternion()
    prim_quat_xyzw = (prim_quat_gf.GetReal(), *prim_quat_gf.GetImaginary())

    # Create a plane on the pallet to scatter the boxes on
    plane_scale = (range_size[0] * 0.8, range_size[1] * 0.8, 1)
    plane_pos_gf = prim_tf_gf.ExtractTranslation() + Gf.Vec3d(0, 0, range_size[2])
    plane_rot_euler_deg = quat_to_euler_angles(np.array(prim_quat_xyzw), degrees=True)
    scatter_plane = rep.create.plane(
        scale=plane_scale, position=plane_pos_gf, rotation=plane_rot_euler_deg, visible=False
    )

    cardbox_mats = [
        f"{assets_root_path}/Isaac/Environments/Simple_Warehouse/Materials/MI_PaperNotes_01.mdl",
        f"{assets_root_path}/Isaac/Environments/Simple_Warehouse/Materials/MI_CardBoxB_05.mdl",
    ]

    def scatter_boxes():
        cardboxes = rep.create.from_usd(
            assets_root_path + config["cardbox"]["url"], semantics=[("class", config["cardbox"]["class"])], count=5
        )
        with cardboxes:
            rep.randomizer.scatter_2d(scatter_plane, check_for_collisions=True)
            rep.randomizer.materials(cardbox_mats)
        return cardboxes.node

    rep.randomizer.register(scatter_boxes)


# Register the place cones randomizer graph
def register_cone_placement(forklift_prim, assets_root_path, config):
    # Get the bottom corners of the oriented bounding box (OBB) of the forklift
    bb_cache = create_bbox_cache()
    centroid, axes, half_extent = compute_obb(bb_cache, forklift_prim.GetPrimPath())
    larger_xy_extent = (half_extent[0] * 1.3, half_extent[1] * 1.3, half_extent[2])
    obb_corners = get_obb_corners(centroid, axes, larger_xy_extent)
    bottom_corners = [
        obb_corners[0].tolist(),
        obb_corners[2].tolist(),
        obb_corners[4].tolist(),
        obb_corners[6].tolist(),
    ]

    # Orient the cone using the OBB (Oriented Bounding Box)
    obb_quat = Gf.Matrix3d(axes).ExtractRotation().GetQuaternion()
    obb_quat_xyzw = (obb_quat.GetReal(), *obb_quat.GetImaginary())
    obb_euler = quat_to_euler_angles(np.array(obb_quat_xyzw), degrees=True)

    def place_cones():
        cones = rep.create.from_usd(
            assets_root_path + config["cone"]["url"], semantics=[("class", config["cone"]["class"])]
        )
        with cones:
            rep.modify.pose(position=rep.distribution.sequence(bottom_corners), rotation_z=obb_euler[2])
        return cones.node

    rep.randomizer.register(place_cones)


# Register light randomization graph
def register_lights_placement(forklift_prim, pallet_prim):
    bb_cache = create_bbox_cache()
    combined_range_arr = compute_combined_aabb(bb_cache, [forklift_prim.GetPrimPath(), pallet_prim.GetPrimPath()])
    pos_min = (combined_range_arr[0], combined_range_arr[1], 6)
    pos_max = (combined_range_arr[3], combined_range_arr[4], 7)

    def randomize_lights():
        lights = rep.create.light(
            light_type="Sphere",
            color=rep.distribution.uniform((0.2, 0.1, 0.1), (0.9, 0.8, 0.8)),
            intensity=rep.distribution.uniform(500, 2000),
            position=rep.distribution.uniform(pos_min, pos_max),
            scale=rep.distribution.uniform(5, 10),
            count=3,
        )
        return lights.node

    rep.randomizer.register(randomize_lights)

def register_randomize_lights(prim_paths):
    bb_cache = create_bbox_cache()
    combined_range_arr = compute_combined_aabb(bb_cache, prim_paths)
    pos_min = (combined_range_arr[0], combined_range_arr[1], (combined_range_arr[5] - combined_range_arr[2]) * 0.7 + combined_range_arr[2])
    pos_max = (combined_range_arr[3], combined_range_arr[4], combined_range_arr[5])
    area_per_light = random.uniform(20, 100)
    count = int((combined_range_arr[3] - combined_range_arr[0]) * (combined_range_arr[4] - combined_range_arr[1]) / area_per_light)
    def randomize_lights_intensities():
        lights = rep.create.light(
            light_type="Sphere",
            color=rep.distribution.uniform((0.5, 0.3, 0.2), (0.9, 0.9, 0.8)),
            intensity=rep.distribution.uniform(300, 10000),
            position=rep.distribution.uniform(pos_min, pos_max),
            scale=rep.distribution.uniform(5, 10),
            count=count,
        )
        return lights.node
    rep.randomizer.register(randomize_lights_intensities)

def register_randomize_sun_light(sky):
    def randomize_sun_light():
        with sky:
            rep.modify.attribute(name="inputs:SunPositionFromTOD",value=True)
            rep.modify.attribute(name="inputs:TimeOfDay",value=rep.distribution.uniform(lower=0.5,upper=23.5))
    rep.randomizer.register(randomize_sun_light)