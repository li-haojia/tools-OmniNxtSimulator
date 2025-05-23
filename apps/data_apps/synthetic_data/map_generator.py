import math
import omni
import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.sim.schemas import RigidBodyPropertiesCfg, CollisionPropertiesCfg
import isaacsim.core.utils.prims as prims_utils
from isaacsim.asset.gen.omap.bindings import _omap
from isaacsim.asset.gen.omap.utils import compute_coordinates, generate_image, update_location
from scipy.spatial import KDTree
import random
from pxr import UsdGeom, Sdf, Gf, Vt
import time

class MapGenerator:
    """
    Class to generate maps with walls and obstacles for drone environments.
    """
    def __init__(self, sim, device="cuda:0"):
        self.device = device
        self.sim = sim
        
    def generate_walls(self, scene):
        """
        Generate walls around the environment.
        
        Args:
            scene: Scene configuration with env_spacing parameter
        
        Returns:
            List of wall primitive paths
        """
        wall_prims = []

        # Walls
        wall_cfg = sim_utils.CuboidCfg(
            size=(scene.env_spacing * 1.5, 0.1, 4.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.4, 0.8)),
            rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False, kinematic_enabled=True),
            collision_props=CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.1),
            activate_contact_sensors=True,
        )
        
        # Left wall
        left_wall_path = "/World/ground/left_wall"
        wall_cfg.func(left_wall_path, wall_cfg, translation=(0.0, scene.env_spacing / 2.0, 2.0))
        wall_prims.append(left_wall_path)
        
        # Right wall
        right_wall_path = "/World/ground/right_wall"
        wall_cfg.func(right_wall_path, wall_cfg, translation=(0.0, -scene.env_spacing / 2.0, 2.0))
        wall_prims.append(right_wall_path)
        
        # Forward wall
        # forward_wall_path = "/World/ground/forward_wall"
        # wall_cfg.func(forward_wall_path, wall_cfg, translation=(scene.env_spacing / 2.0 + 2.0, 0.0, 2.0), orientation=(0.707, 0.0, 0.0, 0.707))
        # wall_prims.append(forward_wall_path)
        
        # Backward wall
        backward_wall_path = "/World/ground/backward_wall"
        wall_cfg.func(backward_wall_path, wall_cfg, translation=(-scene.env_spacing / 2.0 - 5.0, 0.0, 2.0), orientation=(0.707, 0.0, 0.0, 0.707))
        wall_prims.append(backward_wall_path)
        
        # Ceiling
        ceiling_path = "/World/ground/ceiling"
        wall_cfg.func(ceiling_path, wall_cfg.replace(size=(scene.env_spacing * 1.5, scene.env_spacing * 1.5, 0.1)), translation=(0.0, 0.0, 2.5))
        wall_prims.append(ceiling_path)

        # Floor
        # floor_path = "/World/ground/floor"
        # wall_cfg.func(floor_path, wall_cfg.replace(size=(scene.env_spacing * 1.5, scene.env_spacing * 1.5, 0.1), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.33, 0.66))), translation=(0.0, 0.0, -0.05))
        # wall_prims.append(floor_path)
        
        return wall_prims
    
    def generate_obstacles(self, scene, num_obstacles=100, min_distance=1.0, obstacle_size_range=(0.3, 0.8), height_range=(0.5, 3.0)):
        """
        Generate random obstacles using PointInstancer for highly efficient geometry creation.
        
        Args:
            scene: Scene configuration with env_spacing parameter
            num_obstacles: Number of obstacles to generate
            min_distance: Minimum distance between obstacle surfaces
            obstacle_size_range: Range of obstacle sizes (min, max)
            height_range: Range of obstacle heights (min, max)
            
        Returns:
            List of obstacle primitive paths and their positions
        """
        # Environment bounds
        x_min = -scene.env_spacing / 2.0 + 1.0
        x_max = scene.env_spacing / 2.0 - 1.0
        y_min = -scene.env_spacing / 2.0 + 1.0
        y_max = scene.env_spacing / 2.0 - 1.0
        
        # Maximum possible radius to account for largest obstacle
        max_size = obstacle_size_range[1]
        max_radius = math.sqrt((max_size/2)**2 + (max_size/2)**2)  # Diagonal of largest obstacle
        
        # Minimum distance between points (centers)
        min_center_distance = 2 * max_radius + min_distance
        
        # Generate points using Poisson disk sampling
        points = self.poisson_disk_sampling(
            width=x_max - x_min,
            height=y_max - y_min,
            min_distance=min_center_distance,
            max_points=num_obstacles
        )
        
        print(f"Generated {len(points)} points for obstacles")
        
        # Create prototypes for cuboid and cylinder
        cuboid_prototype_path = "/World/ground/prototypes/cuboid_prototype"
        cylinder_prototype_path = "/World/ground/prototypes/cylinder_prototype"
        
        # Create prototype directories if needed
        prototype_dir = "/World/ground/prototypes"
        if not prims_utils.is_prim_path_valid(prototype_dir):
            prims_utils.create_prim(prototype_dir, "Xform")
        
        # Create prototype geometries with standard sizes
        cuboid_cfg = sim_utils.CuboidCfg(
            size=(1.0, 1.0, 1.0),  # Reference unit size
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0)),
            rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False, kinematic_enabled=True),
            collision_props=CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.1),
            activate_contact_sensors=True,
        )
        
        cylinder_cfg = sim_utils.CylinderCfg(
            radius=0.5,  # Reference unit radius
            height=1.0,  # Reference unit height
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0)),
            rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False, kinematic_enabled=True),
            collision_props=CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.1),
            activate_contact_sensors=True,
        )
        
        # Create prototypes
        if not prims_utils.is_prim_path_valid(cuboid_prototype_path):
            cuboid_cfg.func(cuboid_prototype_path, cuboid_cfg)
        if not prims_utils.is_prim_path_valid(cylinder_prototype_path):
            cylinder_cfg.func(cylinder_prototype_path, cylinder_cfg)
        
        # Create PointInstancers
        cuboid_instancer_path = "/World/ground/cuboid_instancer"
        cylinder_instancer_path = "/World/ground/cylinder_instancer"
        
        # Get the stage
        stage = omni.usd.get_context().get_stage()
        
        # Create or get instancers
        if not prims_utils.is_prim_path_valid(cuboid_instancer_path):
            # Create the PointInstancer primitive
            cuboid_instancer = UsdGeom.PointInstancer.Define(stage, cuboid_instancer_path)
            # Add prototype references
            cuboid_instancer.GetPrototypesRel().AddTarget(cuboid_prototype_path)
        
        if not prims_utils.is_prim_path_valid(cylinder_instancer_path):
            # Create the PointInstancer primitive
            cylinder_instancer = UsdGeom.PointInstancer.Define(stage, cylinder_instancer_path)
            # Add prototype references
            cylinder_instancer.GetPrototypesRel().AddTarget(cylinder_prototype_path)
        
        # Prepare transform data for instances
        cuboid_positions = []
        cuboid_scales = []
        cuboid_orientations = []
        cylinder_positions = []
        cylinder_scales = []
        cylinder_orientations = []
        
        # Prepare obstacle info and materials
        obstacle_positions = []
        obstacle_types = []
        cuboid_colors = []
        cylinder_colors = []
        
        # Process all points and prepare instance data
        for i, point in enumerate(points):
            # Translate point to world coordinates
            x = point[0] + x_min
            y = point[1] + y_min
            
            # Generate obstacle dimensions
            width = random.uniform(*obstacle_size_range)
            length = random.uniform(*obstacle_size_range)
            height = random.uniform(*height_range)
            
            # Randomly choose obstacle type
            obstacle_type = random.choice(["cuboid", "cylinder"])
            
            # Calculate color based on height (gradient from red to blue)
            height_ratio = (height - height_range[0]) / (height_range[1] - height_range[0] + 1e-5)
            color = (
                0.9 * (1.0 - height_ratio),  # Red decreases with height
                0.2,                         # Green constant
                0.9 * height_ratio,          # Blue increases with height
            )
            
            # Record obstacle information
            if obstacle_type == "cuboid":
                obstacle_info = (x, y, width, length)
                
                # Add to instancer data
                cuboid_positions.append((x, y, height/2.0 + 0.05))
                cuboid_scales.append((width, length, height))
                cuboid_orientations.append((1.0, 0.0, 0.0, 0.0))
                cuboid_colors.append(color)
            else:  # cylinder
                radius = (width + length) / 4.0
                obstacle_info = (x, y, radius)
                
                # Add to instancer data
                cylinder_positions.append((x, y, height/2.0 + 0.05))
                cylinder_scales.append((radius*2, radius*2, height))  # Scale x2 for diameter
                cylinder_orientations.append((1.0, 0.0, 0.0, 0.0))
                cylinder_colors.append(color)
            
            obstacle_positions.append(obstacle_info)
            obstacle_types.append(obstacle_type)
        
        # Apply all instance transformations at once
        obstacle_prims = []
        
        # Update cuboid instances
        if cuboid_positions:
            # Get the instancer
            cuboid_instancer = UsdGeom.PointInstancer(stage.GetPrimAtPath(cuboid_instancer_path))
            
            # Create proper VtArray for positions, scales and orientations
            # Positions - VtVec3fArray
            pos_vtarray = Vt.Vec3fArray(len(cuboid_positions))
            for i, pos in enumerate(cuboid_positions):
                pos_vtarray[i] = Gf.Vec3f(pos[0], pos[1], pos[2])
            
            # Scales - VtVec3fArray
            scale_vtarray = Vt.Vec3fArray(len(cuboid_scales))
            for i, scale in enumerate(cuboid_scales):
                scale_vtarray[i] = Gf.Vec3f(scale[0], scale[1], scale[2])
            
            # Orientations - VtQuathArray (half-float quaternion)
            orient_vtarray = Vt.QuathArray(len(cuboid_orientations))
            for i, orient in enumerate(cuboid_orientations):
                # Convert to Quaternion with half-precision (Quath)
                orient_vtarray[i] = Gf.Quath(orient[0], orient[1], orient[2], orient[3])
            
            # Set instance indices (all using prototype 0)
            proto_indices = Vt.IntArray(len(cuboid_positions), 0)  # Initialize with zeros
            
            # Set all attributes
            cuboid_instancer.CreateProtoIndicesAttr().Set(proto_indices)
            cuboid_instancer.CreatePositionsAttr().Set(pos_vtarray)
            cuboid_instancer.CreateScalesAttr().Set(scale_vtarray)
            cuboid_instancer.CreateOrientationsAttr().Set(orient_vtarray)
            
            # Add instance paths to output list
            cuboid_count = len(cuboid_positions)
            for i in range(cuboid_count):
                obstacle_prims.append(f"{cuboid_instancer_path}/instance_{i}")
        
        # Update cylinder instances
        if cylinder_positions:
            # Get the instancer
            cylinder_instancer = UsdGeom.PointInstancer(stage.GetPrimAtPath(cylinder_instancer_path))
            
            # Create proper VtArray for positions, scales and orientations
            # Positions - VtVec3fArray
            pos_vtarray = Vt.Vec3fArray(len(cylinder_positions))
            for i, pos in enumerate(cylinder_positions):
                pos_vtarray[i] = Gf.Vec3f(pos[0], pos[1], pos[2])
            
            # Scales - VtVec3fArray
            scale_vtarray = Vt.Vec3fArray(len(cylinder_scales))
            for i, scale in enumerate(cylinder_scales):
                scale_vtarray[i] = Gf.Vec3f(scale[0], scale[1], scale[2])
            
            # Orientations - VtQuathArray (half-float quaternion)
            orient_vtarray = Vt.QuathArray(len(cylinder_orientations))
            for i, orient in enumerate(cylinder_orientations):
                # Convert to Quaternion with half-precision (Quath)
                orient_vtarray[i] = Gf.Quath(orient[0], orient[1], orient[2], orient[3])
            
            # Set instance indices (all using prototype 0)
            proto_indices = Vt.IntArray(len(cylinder_positions), 0)  # Initialize with zeros
            
            # Set all attributes
            cylinder_instancer.CreateProtoIndicesAttr().Set(proto_indices)
            cylinder_instancer.CreatePositionsAttr().Set(pos_vtarray)
            cylinder_instancer.CreateScalesAttr().Set(scale_vtarray)
            cylinder_instancer.CreateOrientationsAttr().Set(orient_vtarray)
            
            # Add instance paths to output list
            cylinder_count = len(cylinder_positions)
            for i in range(cylinder_count):
                obstacle_prims.append(f"{cylinder_instancer_path}/instance_{i}")
        
        return obstacle_prims, obstacle_positions
    
    def poisson_disk_sampling(self, width, height, min_distance, max_points=None, k=30):
        """
        Generate Poisson disk sampling points in a rectangle.
        
        Args:
            width, height: Dimensions of the rectangle
            min_distance: Minimum distance between points
            max_points: Maximum number of points to generate (optional)
            k: Number of attempts to place a new point near an existing one
            
        Returns:
            List of points [(x, y)]
        """
        # Cell size for the grid
        cell_size = min_distance / math.sqrt(2)
        
        # Grid dimensions
        grid_width = int(math.ceil(width / cell_size))
        grid_height = int(math.ceil(height / cell_size))
        
        # Grid to store sample indices - using sparse dictionary for efficiency
        grid = {}
        
        # List of samples
        samples = []
        
        # List of active samples (indices)
        active = []
        
        # Initial sample
        x = random.uniform(0, width)
        y = random.uniform(0, height)
        initial_sample = (x, y)
        
        # Get the grid coordinates for a point
        def get_cell(point):
            return (int(point[0] / cell_size), int(point[1] / cell_size))
        
        # Add first sample
        samples.append(initial_sample)
        active.append(0)
        
        cell_coords = get_cell(initial_sample)
        grid[cell_coords] = 0
        
        # While there are active samples and we haven't reached max_points
        while active and (max_points is None or len(samples) < max_points):
            # Choose a random active sample
            active_index = random.randrange(len(active))
            sample_index = active[active_index]
            sample = samples[sample_index]
            
            # Try to generate a new sample near the active one
            found = False
            for _ in range(k):
                # Generate a random point between min_distance and 2*min_distance from the sample
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(min_distance, 2 * min_distance)
                new_x = sample[0] + math.cos(angle) * distance
                new_y = sample[1] + math.sin(angle) * distance
                
                # Check if the point is in bounds
                if not (0 <= new_x < width and 0 <= new_y < height):
                    continue
                    
                # Check if the point is far enough from existing samples
                cell_x, cell_y = get_cell((new_x, new_y))
                
                # Check nearby cells
                valid = True
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        neighbor_cell = (cell_x + dx, cell_y + dy)
                        
                        if neighbor_cell in grid:
                            neighbor_sample = samples[grid[neighbor_cell]]
                            distance = math.sqrt((new_x - neighbor_sample[0])**2 + (new_y - neighbor_sample[1])**2)
                            if distance < min_distance:
                                valid = False
                                break
                                
                if not valid:
                    continue
                    
                # Add the new sample
                new_sample = (new_x, new_y)
                new_sample_index = len(samples)
                samples.append(new_sample)
                active.append(new_sample_index)
                
                # Add to grid
                grid[get_cell(new_sample)] = new_sample_index
                
                found = True
                break
                
            if not found:
                # Remove the active sample we just processed
                active.pop(active_index)
                
        # If we need to limit the number of points
        if max_points is not None and len(samples) > max_points:
            samples = samples[:max_points]
            
        return samples
    
    def generate_floaters(self, scene, num_floaters=100, min_distance=1.0, floater_size_range=(0.3, 0.8), height_range=(0.5, 3.0)):
        """
        Generate random floating objects using PointInstancer for efficiency.
        Uses stratified sampling to ensure uniform distribution across the entire environment.
        
        Args:
            scene: Scene configuration with env_spacing parameter
            num_floaters: Number of floating objects to generate
            min_distance: Minimum distance between floating objects
            floater_size_range: Range of floating object sizes (min, max)
            height_range: Range of floating object heights above ground (min, max)
            
        Returns:
            List of floater primitive paths and their positions
        """
        # Environment bounds
        x_min = -scene.env_spacing / 2.0 + 1.5
        x_max = scene.env_spacing / 2.0 - 1.5
        y_min = -scene.env_spacing / 2.0 + 1.5
        y_max = scene.env_spacing / 2.0 - 1.5
        z_min = height_range[0]
        z_max = height_range[1]
        
        # Calculate volume dimensions
        volume_width = x_max - x_min
        volume_height = y_max - y_min
        volume_depth = z_max - z_min
        
        # List to store generated points
        points_3d = []
        
        # Create spatial index for distance checks
        spatial_grid = {}
        spatial_cell_size = min_distance
        
        # Helper function to get spatial grid cell for a point
        def get_spatial_cell(pos):
            return (int(pos[0] / spatial_cell_size), 
                    int(pos[1] / spatial_cell_size), 
                    int(pos[2] / spatial_cell_size))
        
        # Helper function to check if a point is valid (far enough from existing points)
        def is_valid_point(pos):
            # Check if point is within bounds
            if not (x_min <= pos[0] < x_max and y_min <= pos[1] < y_max and z_min <= pos[2] < z_max):
                return False
            
            # Get the spatial cell for the point
            cell = get_spatial_cell(pos)
            
            # Check surrounding cells
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    for dz in range(-1, 2):
                        neighbor_cell = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                        if neighbor_cell in spatial_grid:
                            for other_pos in spatial_grid[neighbor_cell]:
                                dist = math.sqrt(sum((a-b)**2 for a, b in zip(pos, other_pos)))
                                if dist < min_distance:
                                    return False
            return True
        
        # Add a valid point to our collection
        def add_point(pos):
            points_3d.append(pos)
            cell = get_spatial_cell(pos)
            if cell not in spatial_grid:
                spatial_grid[cell] = []
            spatial_grid[cell].append(pos)
        
        # Stratified sampling approach
        # Calculate grid dimensions for sampling
        cells_per_dim = max(3, int(math.ceil(num_floaters ** (1/3) * 1.5)))
        total_cells = cells_per_dim ** 3
        
        print(f"Using {cells_per_dim}³ grid for stratified sampling ({total_cells} cells)")
        
        # Cell sizes
        cell_width = volume_width / cells_per_dim
        cell_height = volume_height / cells_per_dim
        cell_depth = volume_depth / cells_per_dim
        
        # Create and shuffle cell indices for randomized processing
        cell_indices = [(i, j, k) 
                        for i in range(cells_per_dim) 
                        for j in range(cells_per_dim) 
                        for k in range(cells_per_dim)]
        random.shuffle(cell_indices)
        
        # Fill cells with stratified sampling
        jitter_factor = 0.8  # Controls jitter amount (0.8 = 80% of cell size)
        cells_to_fill = min(total_cells, num_floaters)
        filled_cells = 0
        
        for i, j, k in cell_indices:
            if filled_cells >= cells_to_fill:
                break
                
            # Calculate cell center
            cell_center = (
                x_min + (i + 0.5) * cell_width,
                y_min + (j + 0.5) * cell_height,
                z_min + (k + 0.5) * cell_depth
            )
            
            # Apply random jitter within the cell
            jitter = (
                random.uniform(-jitter_factor, jitter_factor) * cell_width * 0.5,
                random.uniform(-jitter_factor, jitter_factor) * cell_height * 0.5,
                random.uniform(-jitter_factor, jitter_factor) * cell_depth * 0.5
            )
            
            # Create point with jitter
            pos = (cell_center[0] + jitter[0], 
                   cell_center[1] + jitter[1], 
                   cell_center[2] + jitter[2])
            
            # Add if valid
            if is_valid_point(pos):
                add_point(pos)
                filled_cells += 1
        
        print(f"Stratified sampling: placed {filled_cells}/{cells_to_fill} points")
        
        # Fill any remaining points with random sampling if needed
        if filled_cells < num_floaters:
            remaining = num_floaters - filled_cells
            max_attempts = remaining * 5
            attempts = 0
            added = 0
            
            while added < remaining and attempts < max_attempts:
                pos = (random.uniform(x_min, x_max),
                       random.uniform(y_min, y_max),
                       random.uniform(z_min, z_max))
                
                if is_valid_point(pos):
                    add_point(pos)
                    added += 1
                
                attempts += 1
            
            print(f"Random sampling: added {added}/{remaining} points")
        
        print(f"Generated {len(points_3d)} floater points")
        
        # Create prototypes
        prototype_paths = {
            "sphere": "/World/ground/prototypes/sphere_prototype",
            "capsule": "/World/ground/prototypes/capsule_prototype",
            "cone": "/World/ground/prototypes/cone_prototype"
        }
        
        instancer_paths = {
            "sphere": "/World/ground/sphere_instancer",
            "capsule": "/World/ground/capsule_instancer",
            "cone": "/World/ground/cone_instancer"
        }
        
        # Create prototype directory if needed
        prototype_dir = "/World/ground/prototypes"
        if not prims_utils.is_prim_path_valid(prototype_dir):
            prims_utils.create_prim(prototype_dir, "Xform")
        
        # Create prototype geometries
        prototype_configs = {
            "sphere": sim_utils.SphereCfg(
                radius=0.5,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0)),
                rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False, kinematic_enabled=True),
                collision_props=CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.1),
                activate_contact_sensors=True,
            ),
            "capsule": sim_utils.CapsuleCfg(
                radius=0.25,
                height=1.0,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0)),
                rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False, kinematic_enabled=True),
                collision_props=CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.1),
                activate_contact_sensors=True,
            ),
            "cone": sim_utils.ConeCfg(
                radius=0.5,
                height=1.0,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0)),
                rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False, kinematic_enabled=True),
                collision_props=CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.1),
                activate_contact_sensors=True,
            )
        }
        
        # Create prototypes if they don't exist
        for shape_type, path in prototype_paths.items():
            if not prims_utils.is_prim_path_valid(path):
                prototype_configs[shape_type].func(path, prototype_configs[shape_type])
        
        # Get the stage
        stage = omni.usd.get_context().get_stage()
        
        # Create or get instancers
        for shape_type, path in instancer_paths.items():
            if not prims_utils.is_prim_path_valid(path):
                instancer = UsdGeom.PointInstancer.Define(stage, path)
                instancer.GetPrototypesRel().AddTarget(prototype_paths[shape_type])
        
        # Prepare transform data by shape type
        transform_data = {
            "sphere": {"positions": [], "scales": [], "orientations": []},
            "capsule": {"positions": [], "scales": [], "orientations": []},
            "cone": {"positions": [], "scales": [], "orientations": []}
        }
        
        floater_positions = []
        
        # Process points and prepare instance data
        for pos in points_3d:
            # Random size and shape type
            size = random.uniform(*floater_size_range)
            shape_type = random.choice(["sphere", "capsule", "cone"])
            
            # Random orientation using improved method
            u1, u2, u3 = random.random(), random.random(), random.random()
            qw = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
            qx = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)
            qy = math.sqrt(u1) * math.sin(2 * math.pi * u3)
            qz = math.sqrt(u1) * math.cos(2 * math.pi * u3)
            orientation = (qw, qx, qy, qz)
            
            # Add to appropriate instancer data
            transform_data[shape_type]["positions"].append(pos)
            
            if shape_type == "sphere":
                transform_data[shape_type]["scales"].append((size, size, size))
            elif shape_type == "capsule":
                transform_data[shape_type]["scales"].append((size/2, size/2, size))
            else:  # cone
                transform_data[shape_type]["scales"].append((size, size, size))
                
            transform_data[shape_type]["orientations"].append(orientation)
            floater_positions.append(pos)
        
        # Apply all instance transformations
        floater_prims = []
        
        for shape_type, data in transform_data.items():
            if not data["positions"]:
                continue
                
            instancer = UsdGeom.PointInstancer(stage.GetPrimAtPath(instancer_paths[shape_type]))
            
            # Create VtArrays for the transform data
            pos_array = Vt.Vec3fArray(len(data["positions"]))
            scale_array = Vt.Vec3fArray(len(data["scales"]))
            orient_array = Vt.QuathArray(len(data["orientations"]))
            
            # Fill the arrays
            for i, (pos, scale, orient) in enumerate(zip(
                data["positions"], data["scales"], data["orientations"])):
                pos_array[i] = Gf.Vec3f(*pos)
                scale_array[i] = Gf.Vec3f(*scale)
                orient_array[i] = Gf.Quath(*orient)
            
            # Set instance indices (all using prototype 0)
            proto_indices = Vt.IntArray(len(data["positions"]), 0)
            
            # Set all attributes
            instancer.CreateProtoIndicesAttr().Set(proto_indices)
            instancer.CreatePositionsAttr().Set(pos_array)
            instancer.CreateScalesAttr().Set(scale_array)
            instancer.CreateOrientationsAttr().Set(orient_array)
            
            # Add instance paths to output list
            for i in range(len(data["positions"])):
                floater_prims.append(f"{instancer_paths[shape_type]}/instance_{i}")
        
        return floater_prims, floater_positions
    
    def generate_occupancy_map(self, scene):
        """
        Generate an occupancy map for the environment.
        
        Args:
            scene: Scene configuration with env_spacing parameter
            
        Returns:
            KDTree of occupied points
        """
        # Occupancy map
        physx = omni.physx.acquire_physx_interface()
        stage_id = omni.usd.get_context().get_stage_id()
        occ_generator = _omap.Generator(physx, stage_id)
        
        # Use a larger voxel size for better performance (0.2m → 0.3m)
        # Adjust these parameters based on your environment size and required precision
        voxel_size = scene.voxel_width  # Larger voxel size for better performance

        occ_generator.update_settings(voxel_size, 1, 0, 2)
        
        # Set location to map from and the min and max bounds to map to
        occ_generator.set_transform(
            (0, 0, 0), 
            (-scene.env_spacing / 2.0 - 2, -scene.env_spacing / 2.0 - 2, 0.0), # 0.1m above ground
            (scene.env_spacing / 2.0 + 2, scene.env_spacing / 2.0 + 2, 5)
        )
        
        # Generate the 3D occupancy map
        occ_generator.generate3d()
        points = occ_generator.get_occupied_positions()
        print(f"Raw occupied points: {len(points)}")
        
        # Handle empty points case
        if len(points) == 0:
            print("No points found in the occupancy map.")
            points = np.array([[0, 0, 0]])
            return KDTree(points), points
        
        # Convert to numpy array for faster processing
        points_np = np.array(points)
        
        # Create KDTree for efficient nearest neighbor queries
        kdtree = KDTree(points_np)
        
        return kdtree, points_np

    def generate_ground_plane(self, scene):
        """
        Generate a ground plane for the environment.
        
        Args:
            scene: Scene configuration with env_spacing parameter
            
        Returns:
            Ground plane primitive path
        """
        ground_path = "/World/ground/defaultGroundPlane"
        cfg_ground = sim_utils.GroundPlaneCfg(
            color=(0.1, 0.1, 0.1),
            size=(scene.env_spacing * math.sqrt(scene.num_envs) + 10, scene.env_spacing * math.sqrt(scene.num_envs) + 10)
        )
        cfg_ground.func(ground_path, cfg_ground, translation=(0, 0, 0.01))
        
        return ground_path

    def create_environment(self, scene, num_obstacles=100, min_distance=1.0, obstacle_size_range=(0.3, 0.8), 
                      obstacle_height_range=(0.5, 3.0), num_floaters=50, floaters_size_range=(0.1, 0.5), floaters_height_range=(1.0, 4.0)):
        """
        Create a full environment with ground, walls, obstacles, and floating objects.
        
        Args:
            scene: Scene configuration with env_spacing parameter
            num_obstacles: Number of obstacles to generate
            min_distance: Minimum distance between obstacles
            obstacle_size_range: Range of obstacle sizes (min, max)
            height_range: Range of obstacle heights (min, max)
            num_floaters: Number of floating objects to generate
            
        Returns:
            Dictionary containing:
            - ground_plane: Ground plane primitive path
            - walls: List of wall primitive paths
            - obstacles: List of obstacle primitive paths
            - obstacle_positions: List of obstacle positions
            - floaters: List of floater primitive paths
            - floater_positions: List of floater positions
            - kdtree: KDTree of occupied points
        """
        self.sim.pause()
        print("Generating environment...")
        start_time = time.time()

        # Generate ground plane
        ground_time = time.time()
        ground_plane = self.generate_ground_plane(scene)
        print(f"Ground plane generation: {time.time() - ground_time:.3f} seconds")
        
        # Generate walls
        walls_time = time.time()
        walls = self.generate_walls(scene)
        print(f"Walls generation: {time.time() - walls_time:.3f} seconds")
        
        # Generate obstacles
        obstacles_time = time.time()
        print(f"Generating {num_obstacles} obstacles...")
        obstacles, obstacle_positions = self.generate_obstacles(
            scene, 
            num_obstacles=num_obstacles,
            min_distance=min_distance,
            obstacle_size_range=obstacle_size_range,
            height_range=obstacle_height_range
        )
        print(f"Obstacles generation: {time.time() - obstacles_time:.3f} seconds")
        
        # Generate floating objects
        floaters_time = time.time()
        # Only generate floaters if requested
        if num_floaters > 0:
            print(f"Generating {num_floaters} floating objects...")
            floaters, floater_positions = self.generate_floaters(
                scene,
                num_floaters=num_floaters,
                min_distance=min_distance,
                floater_size_range=floaters_size_range,
                height_range=floaters_height_range
            )
        else:
            floaters, floater_positions = [], []
        print(f"Floaters generation: {time.time() - floaters_time:.3f} seconds")
        
        self.sim.play()
        
        # Generate occupancy map
        occ_time = time.time()
        kdtree, points = self.generate_occupancy_map(scene)
        print(f"Occupancy map generation: {time.time() - occ_time:.3f} seconds")
        
        total_time = time.time() - start_time
        print(f"Total environment generation time: {total_time:.3f} seconds")

        return {
            "ground_plane": ground_plane,
            "walls": walls,
            "obstacles": obstacles,
            "obstacle_positions": obstacle_positions,
            "floaters": floaters,
            "floater_positions": floater_positions,
            "kdtree": kdtree,
            "points": points,
            "generation_time": total_time
        }