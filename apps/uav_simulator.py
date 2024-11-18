import argparse
import threading
import torch
import logging

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
from controller.omninxt import OmniNxt

# Configure logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


class UAVSimulation:
    """Class to handle OMNINXT simulation with ROS2 integration."""

    def __init__(self, simulation_app, quadrotor):
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
        self.quadrotor = quadrotor

        # Initialize ROS2
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node('omninxt_sim')

        # ROS2 Publisher and Subscriber
        self.robot_odom_publisher = self.ros2_node.create_publisher(Odometry, '/odom', 10)
        self.prop_force_subscriber = self.ros2_node.create_subscription(
            Float32MultiArray,
            '/prop_force',
            self.prop_force_callback,
            10
        )
        self.prop_torque_subscriber = self.ros2_node.create_subscription(
            Float32MultiArray,
            '/prop_torque',
            self.prop_torque_callback,
            10
        )

        # Thread synchronization
        self.lock = threading.Lock()
        self.prop_force = torch.zeros(4, 3).to(self.sim.device)
        self.prop_torque = torch.zeros(4, 3).to(self.sim.device)
        self.reset_robot = False
        self.shutdown_event = threading.Event()

        # Load environment and robot
        self.load_environment()
        self.load_robot()

        # Prepare simulation parameters
        self.prepare_simulation()

    def load_environment(self):
        """Load the warehouse environment into the simulation stage."""
        env_usd_path = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
        # env_usd_path = f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"
        add_reference_to_stage(env_usd_path, "/World/Environment")
        logging.info("Environment loaded.")

    def load_robot(self):
        """Spawn the OMNINXT robot into the simulation."""
        robot_cfg = self.quadrotor.ISAAC_SIM_CFG.replace(prim_path="/World/OmniNxt")
        robot_cfg.spawn.func("/World/OmniNxt", robot_cfg.spawn, translation=robot_cfg.init_state.pos)
        self.robot = Articulation(robot_cfg)
        logging.info("OMNINXT robot spawned.")

    def prepare_simulation(self):
        """Initialize simulation parameters and reset the simulation."""
        self.sim.reset()
        self.simulate_initial_robot_state()
        logging.info("Simulation setup complete.")

    def simulate_initial_robot_state(self):
        """Set the initial state of the robot in the simulation."""
        joint_pos, joint_vel = self.robot.data.default_joint_pos, self.robot.data.default_joint_vel
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel)
        self.robot.write_root_pose_to_sim(self.robot.data.default_root_state[:, :7])
        self.robot.write_root_velocity_to_sim(self.robot.data.default_root_state[:, 7:])
        self.robot.reset()

    def prop_force_callback(self, msg: Float32MultiArray):
        """Callback function to handle incoming propeller force messages.

        Args:
            msg (Float32MultiArray): Incoming propeller forces.
        """
        logging.debug(f"Received prop_force: {msg.data}")
        try:
            if len(msg.data) != 4:
                logging.warning("Received prop_force message with unexpected length.")
                return
            self.prop_force = torch.tensor([
                [0.0, 0.0, msg.data[0]],
                [0.0, 0.0, msg.data[1]],
                [0.0, 0.0, msg.data[2]],
                [0.0, 0.0, msg.data[3]]
            ], device=self.sim.device)
            logging.debug(f"prop_force_updated: {self.prop_force}")
        except Exception as e:
            logging.error(f"Error processing prop_force message: {e}")

    def prop_torque_callback(self, msg: Float32MultiArray):
        """Callback function to handle incoming propeller torque messages.

        Args:
            msg (Float32MultiArray): Incoming propeller torques.
        """
        logging.debug(f"Received prop_torque: {msg.data}")
        try:
            if len(msg.data) != 4:
                logging.warning("Received prop_torque message with unexpected length.")
                return
            self.prop_torque = torch.tensor([
                [0.0, 0.0, msg.data[0]],
                [0.0, 0.0, msg.data[1]],
                [0.0, 0.0, msg.data[2]],
                [0.0, 0.0, msg.data[3]]
            ], device=self.sim.device)
            logging.debug(f"prop_torque_updated: {self.prop_torque}")
        except Exception as e:
            logging.error(f"Error processing prop_torque message {e}")

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

    def get_robot_data(self):
        """Retrieve relevant robot data from the simulation.

        Returns:
            dict: Dictionary containing robot position, orientation, and angular velocity.
        """
        robot_data = self.robot.data.root_state_w # [pos:3, quat:4, lin_vel:3, ang_vel:3]
        # Convert robot data to dictionary as numpy arrays
        robot_data_dict = {
            "root_pos_w": robot_data[:, :3].detach()[0],
            "root_quat_w": robot_data[:, 3:7].detach()[0],
            "root_lin_vel_w": robot_data[:, 7:10].detach()[0],
            "root_ang_vel_w": robot_data[:, 10:].detach()[0]
        }
        return robot_data_dict

    def send_robot_data_by_ros2(self, robot_data_dict):
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

        self.robot_odom_publisher.publish(msg)
        logging.debug("Published Odometry message.")

    def reset_robot_state(self):
        """Reset the robot's state in the simulation."""
        self.simulate_initial_robot_state()
        self.prop_force = torch.zeros(4, 3).to(self.sim.device)
        self.prop_torque = torch.zeros(4, 3).to(self.sim.device)
        self.reset_robot = False
        logging.info("Robot state has been reset.")

    def run_simulation_loop(self):
        """Run the main simulation loop."""
        sim_dt = self.sim.get_physics_dt()
        sim_time = 0.0
        count = 0

        # Simulation step preparation
        joint_pos, joint_vel = self.robot.data.default_joint_pos, self.robot.data.default_joint_vel
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel)
        self.robot.write_root_pose_to_sim(self.robot.data.default_root_state[:, :7])
        self.robot.write_root_velocity_to_sim(self.robot.data.default_root_state[:, 7:])
        self.robot.reset()

        # Physics parameters
        prop_body_ids = self.robot.find_bodies("rotor.*")[0]
        robot_mass = self.robot.root_physx_view.get_masses().sum()
        gravity = torch.tensor(self.sim.cfg.gravity, device=self.sim.device).norm()

        logging.info("Starting simulation loop.")

        try:
            while self.simulation_app.is_running():
                # Retrieve current robot data
                robot_data_dict = self.get_robot_data()

                # Apply external forces and torques
                forces = self.prop_force.clone().unsqueeze(0)  # Shape: [1, 4, 3]
                torques = self.prop_torque.clone().unsqueeze(0)  # Shape: [1, 4, 3]

                self.robot.set_external_force_and_torque(forces, torques, body_ids=prop_body_ids)
                self.robot.write_data_to_sim()

                # Step the simulation
                self.sim.step()
                sim_time += sim_dt
                count += 1

                # Update robot state
                self.robot.update(sim_dt)

                # Handle robot reset if requested
                if self.reset_robot:
                    self.reset_robot_state()

                # Publish robot data to ROS2
                self.quadrotor.update_state(
                    robot_data_dict["root_pos_w"].cpu().numpy(),
                    robot_data_dict["root_quat_w"].cpu().numpy(),
                    robot_data_dict["root_lin_vel_w"].cpu().numpy(),
                    robot_data_dict["root_ang_vel_w"].cpu().numpy()
                )
                self.send_robot_data_by_ros2(robot_data_dict)
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

def main():
    """Main entry point for the simulation."""
    quadrotor = OmniNxt()
    simulation = UAVSimulation(simulation_app, quadrotor)
    simulation.run()


if __name__ == "__main__":
    main()