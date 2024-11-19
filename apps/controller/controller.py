import rclpy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import threading

from .uav_controller import UAVController

class Controller:
    control_data = Float32MultiArray()

    def __init__(self, ros2_node, vehicle):
        self.quadrotor = vehicle
        self.controller = UAVController(self.quadrotor)
        self.ros2_node = ros2_node

        self.ros2_node.subscription = self.ros2_node.create_subscription(
            Float32MultiArray,
            self.quadrotor.name_space + '/control_cmd',
            self.control_callback,
            10
        )

        self.force_publisher = self.ros2_node.create_publisher(Float32MultiArray, self.quadrotor.name_space + '/prop_force', 10)
        self.torque_publisher = self.ros2_node.create_publisher(Float32MultiArray, self.quadrotor.name_space + '/prop_torque', 10)
        self.debug_angular_velocity_publisher = self.ros2_node.create_publisher(Odometry, self.quadrotor.name_space + '/debug_angular_velocity', 10)

    def control_callback(self, msg):
        self.control_data = msg

    def run_control(self):
        position_control_flag = False
        angular_velocity_control_flag = False

        if len(self.control_data.data) == 3:
            position_control_flag = True
        elif len(self.control_data.data) == 4:
            angular_velocity_control_flag = True

        if position_control_flag:
            forces, torques = self.controller.position_control(self.control_data.data[0], self.control_data.data[1], self.control_data.data[2])
        elif angular_velocity_control_flag:
            forces, torques = self.controller.angular_velocity_control(self.control_data.data[0], self.control_data.data[1], self.control_data.data[2], self.control_data.data[3])
        else:
            forces = np.zeros(4)
            torques = np.zeros(4)

        forces = [max(min(float(force), 10.0), -10.0) for force in forces]
        torques = [max(min(float(torque), 10.0), -10.0) for torque in torques]

        # Check if the forces and torques are valid
        if np.isnan(forces).any() or np.isnan(torques).any() or len(forces) != 4 or len(torques) != 4 or type(forces[0]) != float or type(torques[0]) != float:
            print(type(forces[0]))
            print(type(torques[0]))
            print(forces)
            print(torques)
            print("Invalid control output. Please check the control algorithm.")
            return

        self.quadrotor.update_control(forces, torques)

        force_msg = Float32MultiArray()
        force_msg.data = forces
        self.force_publisher.publish(force_msg)

        torque_msg = Float32MultiArray()
        torque_msg.data = torques
        self.torque_publisher.publish(torque_msg)

        # Publish angular velocity for debugging
        debug_angular_velocity = Odometry()
        x, y, z = self.controller.so3_control.get_computed_angular_velocity()
        debug_angular_velocity.twist.twist.angular.x = x
        debug_angular_velocity.twist.twist.angular.y = y
        debug_angular_velocity.twist.twist.angular.z = z
        self.debug_angular_velocity_publisher.publish(debug_angular_velocity)
