import argparse
import json
import os
import yaml
import numpy as np
import carb

# Default config for trajectory generation
default_config = {
    "data_path": "data_apps/assets/random",
    "trajectory_name": "generated_trajectories.h5",
    "num_trajectories": 100,
    "voxel_width": 0.1,
    "planner_config": {
        "dilate_radius": 0.2,
        "map_bound": [-6.0, 6.0, -6.0, 6.0, 0.0, 3.0],
        "timeout_rrt": 2.0,
        "max_vel_mag": 3.0,
        "max_tilt_angle": 35.0,  # in degrees
        "data_gen_goal_bound": [-4.0, 4.0, -4.0, 4.0, 0.5, 2.5],
        "seed": 42,
        "minTrajDuration": 2.0,
        "minDist": 2.0
    },
    "pcd_name": "generated_environment.pcd",
}

def generate_trajectories(config):
    """Generate trajectories using the point cloud"""
    print("Generating trajectories...")
    
    pcd_input_path = os.path.join(config["data_path"], config["pcd_name"])
    # Check if point cloud exists
    if not os.path.exists(pcd_input_path):
        print(f"Error: Point cloud file {pcd_input_path} does not exist.")
        print("Please run generate_environment.py first to create the point cloud.")
        return False
    
    try:
        import gcopter as gpb
    except ImportError:
        print("Error: Failed to import gcopter module. Please make sure it's installed.")
        return False
    
    # Create planner config
    print("Creating PlannerConfig...")
    planner_config = gpb.PlannerConfig()
    
    # Set planner parameters from config
    pc = config["planner_config"]
    planner_config.voxelWidth = config["voxel_width"]
    planner_config.dilateRadius = pc["dilate_radius"]
    planner_config.mapBound = pc["map_bound"]
    planner_config.timeoutRRT = pc["timeout_rrt"]
    planner_config.maxVelMag = pc["max_vel_mag"]
    planner_config.maxTiltAngle = np.deg2rad(pc["max_tilt_angle"])
    planner_config.dataGenGoalBound = pc["data_gen_goal_bound"]
    planner_config.seed = pc["seed"]
    planner_config.minTrajDuration = pc["minTrajDuration"]
    planner_config.minDist= pc["minDist"]
    print("Creating PathPlanner...")
    planner = gpb.PathPlanner(planner_config)
    
    # Load the point cloud map
    print(f"Loading map from {pcd_input_path}...")
    if not planner.loadMap(pcd_input_path):
        print("Failed to load point cloud map.")
        return False
    
    # Create trajectory data generator
    print("Creating TrajectoryDataGenerator...")
    data_generator = gpb.TrajectoryDataGenerator(planner_config, planner)
    
    # Generate trajectories
    num_trajectories = config.get("num_trajectories", 100)
    output_path = os.path.join(config["data_path"], config["trajectory_name"])
    
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    print(f"Generating {num_trajectories} trajectories...")
    if data_generator.generateAndSaveTrajectories(num_trajectories, output_path, random_yaw=True):
        print(f"Successfully generated and saved {num_trajectories} trajectories to {output_path}")
        if os.path.exists(output_path):
            print(f"Trajectory file {output_path} created successfully.")
            return True
        else:
            print(f"Error: Trajectory file was not created at {output_path}")
            return False
    else:
        print("Failed to generate trajectories.")
        return False

def main():
    parser = argparse.ArgumentParser(description="Generate trajectories for robot navigation")
    parser.add_argument("--config", required=False, help="Path to config file (json or yaml)")
    args = parser.parse_args()

    config = default_config.copy()
    if args.config and os.path.isfile(args.config):
        with open(args.config, "r") as f:
            if args.config.endswith(".json"):
                config.update(json.load(f))
            elif args.config.endswith(".yaml"):
                config.update(yaml.safe_load(f))

    if generate_trajectories(config):
        print("Trajectory generation completed successfully!")
    else:
        print("Trajectory generation failed!")
        exit(1)

if __name__ == "__main__":
    main() 