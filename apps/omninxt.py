import argparse
import torch

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates how to simulate a quadcopter.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omninxt.omninxt import OMNINXT_CFG

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

def main():
    """Main function."""

    # Load kit helper
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005))
    # Set main camera
    sim.set_camera_view(eye=[0.5, 0.5, 1.0], target=[0.0, 0.0, 0.5])

    # Spawn things into stage
    # Scene
    warehouse_usd_path = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    add_reference_to_stage(warehouse_usd_path, "/World/Warehouse")

    # Robots
    robot_cfg = OMNINXT_CFG.replace(prim_path="/World/OmniNxt")
    robot_cfg.spawn.func("/World/OmniNxt", robot_cfg.spawn, translation=robot_cfg.init_state.pos)

    # create handles for the robots
    robot = Articulation(robot_cfg)

    # Play the simulator
    sim.reset()

    # Fetch relevant parameters to make the quadcopter hover in place
    prop_body_ids = robot.find_bodies("rotor.*")[0]
    robot_mass = robot.root_physx_view.get_masses().sum()
    gravity = torch.tensor(sim.cfg.gravity, device=sim.device).norm()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    joint_pos, joint_vel = robot.data.default_joint_pos, robot.data.default_joint_vel
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.write_root_pose_to_sim(robot.data.default_root_state[:, :7])
    robot.write_root_velocity_to_sim(robot.data.default_root_state[:, 7:])
    robot.reset()

    while simulation_app.is_running():
        # State
        print(f"robot_mass: {robot_mass}")
        robot_data = robot.data
        robot_linear_accel = robot_data.body_lin_acc_w
        print(f"robot_linear_accel: {robot_linear_accel}")
        robot_angular_vel = robot_data.body_ang_vel_w
        print(f"robot_angular_vel: {robot_angular_vel}")
        robot_pos = robot_data.root_pos_w
        print(f"robot_pos: {robot_pos}")
        robot_rot = robot_data.root_quat_w
        print(f"robot_rot: {robot_rot}")

        # apply action to the robot (make the robot float in place)
        forces = torch.zeros(robot.num_instances, 4, 3, device=sim.device)
        torques = torch.zeros_like(forces)
        forces[..., 2] = robot_mass * gravity / 4.0

        # print(f"forces: {forces}")
        # print(f"torques: {torques}")

        robot.set_external_force_and_torque(forces, torques, body_ids=prop_body_ids)
        robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        robot.update(sim_dt)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
