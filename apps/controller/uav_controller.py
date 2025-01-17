import numpy as np
import math

from .so3_control import SO3Control

class UAVController:
    quadopter = None

    # Position control parameters
    kx = np.array([5, 5, 6])
    kv = np.array([3.4, 3.4, 4.0])

    def __init__(self, quadopter):
        self.quadopter = quadopter
        self.so3_control = SO3Control(self.quadopter.mass_, self.quadopter.gravity_)
        self.so3_control.set_mass(self.quadopter.mass_)
        self.so3_control.set_gravity(self.quadopter.gravity_)

    def position_control(self, x, y, z, yaw):
        self.so3_control.set_position(self.quadopter.position_)
        self.so3_control.set_quaternion(self.quadopter.quaternion_[1],
                                        self.quadopter.quaternion_[2],
                                        self.quadopter.quaternion_[3],
                                        self.quadopter.quaternion_[0])
        self.so3_control.set_velocity(self.quadopter.linear_velocity_)
        self.so3_control.set_angular_velocity(self.quadopter.angular_velocity_)
        self.so3_control.calculate_control([x, y, z], [0, 0, 0], yaw, self.kx, self.kv)
        throttle = self.so3_control.get_computed_throttle()
        tau_x, tau_y, tau_z = self.so3_control.get_computed_angular_velocity()
        forces, torques = self.get_px4_forces(throttle, tau_x, tau_y, tau_z)
        return forces, torques

    def velocity_control(self, vx, vy, vz, yaw):
        self.so3_control.set_position(self.quadopter.position_)
        self.so3_control.set_quaternion(self.quadopter.quaternion_[1],
                                        self.quadopter.quaternion_[2],
                                        self.quadopter.quaternion_[3],
                                        self.quadopter.quaternion_[0])
        self.so3_control.set_velocity(self.quadopter.linear_velocity_)
        self.so3_control.set_angular_velocity(self.quadopter.angular_velocity_)
        self.so3_control.calculate_control([np.nan, np.nan, np.nan], [vx, vy, vz], yaw, self.kx, self.kv)
        throttle = self.so3_control.get_computed_throttle()
        tau_x, tau_y, tau_z = self.so3_control.get_computed_angular_velocity()
        forces, torques = self.get_px4_forces(throttle, tau_x, tau_y, tau_z)
        return forces, torques

    def angular_velocity_control(self, ang_vel_x, ang_vel_y, ang_vel_z, throttle):
        forces, torques = self.get_px4_forces(throttle, ang_vel_x, ang_vel_y, ang_vel_z)
        return forces, torques

    def allocate_forces_and_torques(self, arm_length, throttle, omega_x, omega_y, omega_z, k=0.1):
        """
        Allocates the required forces and torques to each motor of an X-shaped quadrotor.

        Parameters:
        - arm_length (float): Length of the arm from the center to each motor in meters.
        - throttle (float): Throttle input.
        - omega_x (float): Desired angular velocity around the x-axis in rad/s (roll rate).
        - omega_y (float): Desired angular velocity around the y-axis in rad/s (pitch rate).
        - omega_z (float): Desired angular velocity around the z-axis in rad/s (yaw rate).
        - k (float): Torque coefficient (default: 1e-6 N.m/N). Adjust based on motor characteristics.

        Returns:
        - dict: A dictionary containing the forces and torques for each motor.
                Keys are 'left_front', 'left_back', 'right_front', 'right_back'.
                Each key maps to another dictionary with 'force' and 'torque_z'.
        """
        # Get current omega values
        current_omega_x = self.quadopter.angular_velocity_[0]
        current_omega_y = self.quadopter.angular_velocity_[1]
        current_omega_z = self.quadopter.angular_velocity_[2]

        # Calculate the omega error
        omega_x = omega_x - current_omega_x
        omega_y = omega_y - current_omega_y
        omega_z = omega_z - current_omega_z

        # Total thrust required
        F_total = throttle

        # Proportional gains for mapping angular velocities to desired torques
        # torques = omega * moments_of_inertia * omega_dot
        Kx = 2.64e-1  # Gain for roll
        Ky = 2.64e-1  # Gain for pitch
        Kz = 5.28e-1  # Gain for yaw

        dx = Kx * 0.03
        dy = Ky * 0.03
        dz = Kz * 0.03

        # Desired torques
        tau_x = Kx * omega_x - dx * current_omega_x  # Roll torque
        tau_y = Ky * omega_y - dy * current_omega_y  # Pitch torque
        tau_z = Kz * omega_z - dz * current_omega_z  # Yaw torque

        max_torque_z = 0.1
        min_torque_z = -0.1
        tau_z = max(min(tau_z, max_torque_z), min_torque_z)

        # Calculate arm length components for X configuration
        # In X configuration, the thrust direction is at 45 degrees to the body axes
        # Hence, the lever arm for roll and pitch is l / sqrt(2)
        l = arm_length / np.sqrt(2)

        # Define the allocation matrix A
        A = np.array([
            [1, 1, 1, 1],           # Total thrust
            [l,  l, -l, -l],         # Roll torque
            [-l, l, -l,  l],         # Pitch torque
            [-k, k, k, -k]        # Yaw torque (reaction torque)
        ])

        # Define the desired outputs vector b
        b = np.array([F_total, tau_x, tau_y, tau_z])

        try:
            # Solve the linear system A * F = b for F (forces)
            F = np.linalg.solve(A, b)

            # Define motor rotation directions
            # Assuming:
            # - left_front and right_front rotate clockwise (positive torque)
            # - left_back and right_back rotate counter-clockwise (negative torque)
            rotation_directions = {
                'left_front':  1,  # Clockwise
                'left_back' : -1,  # Counter-Clockwise
                'right_front': -1,  # Clockwise
                'right_back' :  1   # Counter-Clockwise
            }

            # Assign forces to each motor with appropriate labels
            motor_names = ['left_front', 'left_back', 'right_front', 'right_back']
            allocation = {}
            for i, motor in enumerate(motor_names):
                torque_z = rotation_directions[motor] * k * F[i]
                allocation[motor] = {
                    'force': F[i],
                    'torque_z': torque_z
                }

            return allocation

        except np.linalg.LinAlgError:
            # Handle cases where the matrix A is singular
            raise ValueError("The allocation matrix is singular and cannot be inverted. Check input parameters.")


    def get_px4_forces(self, throttle, tau_x, tau_y, tau_z):
        allocation = self.allocate_forces_and_torques(
            self.quadopter.arm_length_,
            throttle,
            tau_x,
            tau_y,
            tau_z
        )
        forces = np.array([allocation['right_front']['force'], allocation['left_back']['force'], allocation['left_front']['force'], allocation['right_back']['force']])
        torques = np.array([allocation['right_front']['torque_z'], allocation['left_back']['torque_z'], allocation['left_front']['torque_z'], allocation['right_back']['torque_z']])

        return forces, torques