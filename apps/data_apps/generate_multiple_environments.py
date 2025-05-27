import argparse
import os
import subprocess
import json
import sys
import yaml
from pathlib import Path

def load_base_config(config_path):
    """Load the base configuration from yaml file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def create_config(base_path, env_id, base_config):
    """Create config for each script with the appropriate paths"""
    
    # Format environment ID as 4-digit string (e.g., 0000, 0001, etc.)
    env_dir = f"{env_id:04d}"
    env_path = os.path.join(base_path, env_dir)
    
    # Create deep copies of the base config for each script
    env_config = base_config.copy()
    traj_config = base_config.copy()
    image_config = base_config.copy()
    
    # Update only the path-related configurations for each script
    env_config.update({
        "data_path": env_path,
    })
    
    traj_config.update({
        "data_path": env_path,
    })
    
    image_config.update({
        "data_path": env_path,
    })
    
    return env_config, traj_config, image_config

def run_script(script_path, config):
    """Run a Python script with the given config"""
    
    # Create temporary config file
    config_path = f"temp_config_{os.path.basename(script_path)}.json"
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    
    try:
        # Run the script with the config
        cmd = [sys.executable, script_path, "--config", config_path]
        print(f"Running command: {' '.join(cmd)}")
        result = subprocess.run(cmd, check=True)
        print(f"Script {script_path} completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error running script {script_path}: {e}")
        return False
    finally:
        # Clean up temporary config file
        if os.path.exists(config_path):
            os.remove(config_path)

def main():
    parser = argparse.ArgumentParser(description="Generate multiple environments and training data")
    parser.add_argument("--output_dir", required=True, help="Base directory for all generated environments")
    parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to generate")
    parser.add_argument("--config", default="apps/data_apps/configs/random_scene_config.yaml", 
                       help="Path to base configuration file (yaml)")
    args = parser.parse_args()
    
    # Create base output directory if it doesn't exist
    base_path = os.path.abspath(args.output_dir)
    os.makedirs(base_path, exist_ok=True)
    
    # Load base configuration
    base_config = load_base_config(args.config)
    
    # Get the directory where the scripts are located
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Define paths to the three scripts
    env_script = os.path.join(current_dir, "generate_environment.py")
    traj_script = os.path.join(current_dir, "generate_trajectories.py")
    image_script = os.path.join(current_dir, "path_tracking_image_generator.py")
    
    # Generate environments and data
    for env_id in range(args.num_envs):
        print(f"\nProcessing environment {env_id:04d}")
        
        # Create configs for this environment
        env_config, traj_config, image_config = create_config(base_path, env_id, base_config)
        
        # Create environment directory
        env_dir = os.path.join(base_path, f"{env_id:04d}")
        os.makedirs(env_dir, exist_ok=True)
        
        # Run the three scripts in sequence
        if not run_script(env_script, env_config):
            print(f"Failed to generate environment {env_id:04d}, skipping to next")
            continue
            
        if not run_script(traj_script, traj_config):
            print(f"Failed to generate trajectories for environment {env_id:04d}, skipping to next")
            continue
            
        if not run_script(image_script, image_config):
            print(f"Failed to generate images for environment {env_id:04d}")
            continue
            
        print(f"Successfully completed environment {env_id:04d}")

if __name__ == "__main__":
    main() 