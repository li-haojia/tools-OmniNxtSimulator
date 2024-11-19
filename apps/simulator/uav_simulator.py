import threading
import torch
import logging
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

import omni.isaac.lab.sim as sim_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.app import AppLauncher

from controller.controller import Controller

class UAVSimulation:
    """Class to handle OMNINXT simulation with ROS2 integration."""

    def __init__(self, simulation_app, quadrotors = []):
        """Initialize the simulation and ROS2 components.

        Args:
            simulation_app: Simulation application instance.
        """
        # Check if the GPU is available
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available. Please check your NVIDIA driver installation.")

        # Initialize simulation application
        self.simulation_app = simulation_app

        # Initialize simulation context
        self.sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005))
        self.sim.set_camera_view(eye=[0.5, 0.5, 1.0], target=[0.0, 0.0, 0.5])

        # Add quadrotor to the simulation
        self.quadrotors = quadrotors
        self.robot_num = len(self.quadrotors)
        if(self.robot_num == 0):
            logging.warning("No robots added to the simulation.")

        # Initialize ROS2
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node('omninxt_sim')

        # Load environment
        self.load_environment()

        # Thread synchronization
        self.lock = threading.Lock()
        self.reset_robot = False
        self.shutdown_event = threading.Event()

        # Initialize robots
        self.initialize_robots()

        # Prepare simulation parameters
        self.prepare_simulation()

    def initialize_robots(self):
        # Robots initialization
        for quadrotor in self.quadrotors:
            # ROS2 Publisher and Subscriber
            quadrotor.ros2_odom_publisher_ = self.ros2_node.create_publisher(Odometry, quadrotor.name_space + '/odom', 10)

            # Load robot into simulation
            robot_cfg = quadrotor.ISAAC_SIM_CFG.replace(prim_path=f"/World{quadrotor.name_space}/OmniNxt")
            robot_cfg.spawn.func(f"/World{quadrotor.name_space}/OmniNxt", robot_cfg.spawn, translation=robot_cfg.init_state.pos)
            quadrotor.robot_instance_ = Articulation(robot_cfg)
            logging.info(f"OMNINXT robot_{quadrotor.id} spawned.")

            # Initialize robot controller
            quadrotor.controller_instance_ = Controller(self.ros2_node, quadrotor)

    def load_environment(self):
        """Load the warehouse environment into the simulation stage."""
        env_usd_path = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
        # env_usd_path = f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"
        add_reference_to_stage(env_usd_path, "/World/Environment")
        logging.info("Environment loaded.")

    def prepare_simulation(self):
        """Initialize simulation parameters and reset the simulation."""
        self.sim.reset()
        self.simulate_initial_robot_state()
        logging.info("Simulation setup complete.")

    def simulate_initial_robot_state(self):
        """Set the initial state of the robot in the simulation."""
        for quadrotor in self.quadrotors:
            joint_pos, joint_vel = quadrotor.robot_instance_.data.default_joint_pos, quadrotor.robot_instance_.data.default_joint_vel
            quadrotor.robot_instance_.write_joint_state_to_sim(joint_pos, joint_vel)
            quadrotor.robot_instance_.write_root_pose_to_sim(quadrotor.robot_instance_.data.default_root_state[:, :7])
            quadrotor.robot_instance_.write_root_velocity_to_sim(quadrotor.robot_instance_.data.default_root_state[:, 7:])
            quadrotor.prop_forces = []
            quadrotor.prop_torques = []
            quadrotor.robot_instance_.reset()

    def ros_spin(self):
        """Spin the ROS2 node to process callbacks."""
        try:
            rclpy.spin(self.ros2_node)
        except Exception as e:
            logging.error(f"ROS2 spin encountered an error: {e}")

    def user_input_handler(self):
        """Handle user input for resetting the robot."""
        while not self.shutdown_event.is_set():
            try:
                user_input = input('Please enter "R" to reset the robot or "Q" to quit: ').strip().lower()
                if user_input == 'r':
                    self.reset_robot = True
                    logging.info("Reset command received.")
                elif user_input == 'q':
                    logging.info("Quit command received. Shutting down simulation.")
                    self.shutdown()
                else:
                    logging.warning('Invalid input. Please enter "R" to reset the robot.')
            except EOFError:
                logging.info("EOF received. Shutting down user input thread.")
                break
            except Exception as e:
                logging.error(f"Error in user input thread: {e}")
                break

    def get_robot_data(self, robot):
        """Retrieve relevant robot data from the simulation.

        Returns:
            dict: Dictionary containing robot position, orientation, and angular velocity.
        """
        robot_data = robot.data.root_state_w # [pos:3, quat:4, lin_vel:3, ang_vel:3]
        # Convert robot data to dictionary as numpy arrays
        robot_data_dict = {
            "root_pos_w": robot_data[:, :3].detach()[0],
            "root_quat_w": robot_data[:, 3:7].detach()[0],
            "root_lin_vel_w": robot_data[:, 7:10].detach()[0],
            "root_ang_vel_w": robot_data[:, 10:].detach()[0]
        }
        return robot_data_dict

    def send_robot_data_by_ros2(self, publisher, robot_data_dict):
        """Publish the robot's odometry data to ROS2.

        Args:
            robot_data_dict (dict): Dictionary containing robot data.
        """
        msg = Odometry()
        msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        # Position
        msg.pose.pose.position.x = robot_data_dict["root_pos_w"][0].item()
        msg.pose.pose.position.y = robot_data_dict["root_pos_w"][1].item()
        msg.pose.pose.position.z = robot_data_dict["root_pos_w"][2].item()

        # Orientation with quaternion transformation
        msg.pose.pose.orientation.x = robot_data_dict["root_quat_w"][1].item()
        msg.pose.pose.orientation.y = robot_data_dict["root_quat_w"][2].item()
        msg.pose.pose.orientation.z = robot_data_dict["root_quat_w"][3].item()
        msg.pose.pose.orientation.w = robot_data_dict["root_quat_w"][0].item()

        # Linear velocity
        msg.twist.twist.linear.x = robot_data_dict["root_lin_vel_w"][0].item()
        msg.twist.twist.linear.y = robot_data_dict["root_lin_vel_w"][1].item()
        msg.twist.twist.linear.z = robot_data_dict["root_lin_vel_w"][2].item()

        # Angular velocity
        msg.twist.twist.angular.x = robot_data_dict["root_ang_vel_w"][0].item()
        msg.twist.twist.angular.y = robot_data_dict["root_ang_vel_w"][1].item()
        msg.twist.twist.angular.z = robot_data_dict["root_ang_vel_w"][2].item()

        publisher.publish(msg)
        logging.debug("Published Odometry message.")

    def reset_robot_state(self):
        """Reset the robot's state in the simulation."""
        self.simulate_initial_robot_state()
        self.reset_robot = False
        logging.info("Robot state has been reset.")

    def run_simulation_loop(self):
        """Run the main simulation loop."""
        logging.info("Starting simulation loop.")

        try:
            while self.simulation_app.is_running():

                # Apply external forces and torques
                for quadrotor in self.quadrotors:
                    quadrotor.controller_instance_.run_control()
                    forces = torch.zeros((1, 4, 3), device=self.sim.device)
                    torques = torch.zeros_like(forces)

                    forces[0, :, 2] = torch.tensor(quadrotor.prop_forces)
                    torques[0, :, 2] = torch.tensor(quadrotor.prop_torques)

                    prop_body_ids = quadrotor.robot_instance_.find_bodies("rotor.*")[0]
                    quadrotor.robot_instance_.set_external_force_and_torque(forces, torques, body_ids=prop_body_ids)
                    quadrotor.robot_instance_.write_data_to_sim()

                # Step the simulation
                self.sim.step()
                sim_dt = self.sim.get_physics_dt()

                for quadrotor in self.quadrotors:
                    quadrotor.robot_instance_.update(sim_dt)
                    # Retrieve current robot data
                    robot_data_dict = self.get_robot_data(quadrotor.robot_instance_)
                    # Publish robot data to ROS2
                    quadrotor.update_state(
                        robot_data_dict["root_pos_w"].cpu().numpy(),
                        robot_data_dict["root_quat_w"].cpu().numpy(),
                        robot_data_dict["root_lin_vel_w"].cpu().numpy(),
                        robot_data_dict["root_ang_vel_w"].cpu().numpy()
                    )
                    self.send_robot_data_by_ros2(quadrotor.ros2_odom_publisher_, robot_data_dict)

                # Handle robot reset if requested
                if self.reset_robot:
                    self.reset_robot_state()

        except KeyboardInterrupt:
            logging.info("Simulation interrupted by user.")
        except Exception as e:
            logging.error(f"An error occurred during simulation: {e}")
        finally:
            self.shutdown()

    def start_ros_spin_thread(self):
        """Start a separate thread to spin the ROS2 node."""
        self.ros_thread = threading.Thread(target=self.ros_spin, name="ROS2_Spin_Thread", daemon=True)
        self.ros_thread.start()
        logging.info("ROS2 spin thread started.")

    def start_user_input_thread(self):
        """Start a separate thread to handle user input."""
        self.user_input_thread = threading.Thread(target=self.user_input_handler, name="UserInput_Thread", daemon=True)
        self.user_input_thread.start()
        logging.info("User input thread started.")

    def shutdown(self):
        """Shutdown simulation and ROS2 components gracefully."""
        logging.info("Shutting down simulation and ROS2.")
        self.shutdown_event.set()

        # Shutdown ROS2
        rclpy.shutdown()
        if hasattr(self, 'ros2_node') and self.ros2_node is not None:
            self.ros2_node.destroy_node()

        # Join threads
        if hasattr(self, 'ros_thread') and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
        if hasattr(self, 'user_input_thread') and self.user_input_thread.is_alive():
            self.user_input_thread.join(timeout=1.0)

        # Close simulation app
        if self.simulation_app is not None:
            self.simulation_app.close()
        logging.info("Shutdown complete.")

    def run(self):
        """Run the entire simulation setup."""
        self.start_ros_spin_thread()
        self.start_user_input_thread()
        self.run_simulation_loop()
