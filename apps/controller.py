import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import threading

from controller.uav_controller import UAVController
from controller.omninxt import OmniNxt

class ControllerNode(Node):
    dt = 0.01 # Control loop period
    odom_data = Odometry()
    control_data = Float32MultiArray()

    def __init__(self):
        super().__init__('controller_node')
        self.quadrotor = OmniNxt()
        self.controller = UAVController(self.quadrotor)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/control_cmd',
            self.control_callback,
            10
        )

        self.force_publisher = self.create_publisher(Float32MultiArray, '/prop_force', 10)
        self.torque_publisher = self.create_publisher(Float32MultiArray, '/prop_torque', 10)
        self.debug_angular_velocity_publisher = self.create_publisher(Odometry, '/debug_angular_velocity', 10)

        # Use a timer instead of publishing directly in the callback
        self.odom_data = None
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg):
        self.odom_data = msg
        position = np.array([self.odom_data.pose.pose.position.x,
                            self.odom_data.pose.pose.position.y,
                            self.odom_data.pose.pose.position.z])
        quaternion = np.array([self.odom_data.pose.pose.orientation.w,
                            self.odom_data.pose.pose.orientation.x,
                            self.odom_data.pose.pose.orientation.y,
                            self.odom_data.pose.pose.orientation.z])
        linear_velocity = np.array([self.odom_data.twist.twist.linear.x,
                                    self.odom_data.twist.twist.linear.y,
                                    self.odom_data.twist.twist.linear.z])
        angular_velocity = np.array([self.odom_data.twist.twist.angular.x,
                                    self.odom_data.twist.twist.angular.y,
                                    self.odom_data.twist.twist.angular.z])
        self.quadrotor.update_state(position, quaternion, linear_velocity, angular_velocity)

    def control_callback(self, msg):
        self.control_data = msg

    def control_loop(self):
        position_control_flag = False
        angular_velocity_control_flag = False
        forces = np.zeros(4)
        torques = np.zeros(4)

        if len(self.control_data.data) == 3:
            position_control_flag = True
        elif len(self.control_data.data) == 4:
            angular_velocity_control_flag = True
        else:
            self.get_logger().warn("Invalid control command. Please enter a 3D position or Throttle with 3D angular velocity.")

        if position_control_flag:
            forces, torques = self.controller.position_control(self.control_data.data[0], self.control_data.data[1], self.control_data.data[2])
        elif angular_velocity_control_flag:
            forces, torques = self.controller.angular_velocity_control(self.control_data.data[0], self.control_data.data[1], self.control_data.data[2], self.control_data.data[3])

        forces = [max(min(float(force), 10.0), 0.0) for force in forces]
        torques = [max(min(float(torque), 10.0), -10.0) for torque in torques]

        # Check if the forces and torques are valid
        if np.isnan(forces).any() or np.isnan(torques).any() or len(forces) != 4 or len(torques) != 4 or type(forces[0]) != float or type(torques[0]) != float:
            print(type(forces[0]))
            print(type(torques[0]))
            print(forces)
            print(torques)
            self.get_logger().warn("Invalid control output. Please check the control algorithm.")
            return

        force_msg = Float32MultiArray()
        force_msg.data = forces
        self.force_publisher.publish(force_msg)

        torque_msg = Float32MultiArray()
        torque_msg.data = torques
        self.torque_publisher.publish(torque_msg)

        # Reset control data
        self.control_data = Float32MultiArray()

        # Publish angular velocity for debugging
        debug_angular_velocity = Odometry()
        x, y, z = self.controller.so3_control.get_computed_angular_velocity()
        debug_angular_velocity.twist.twist.angular.x = x
        debug_angular_velocity.twist.twist.angular.y = y
        debug_angular_velocity.twist.twist.angular.z = z
        self.debug_angular_velocity_publisher.publish(debug_angular_velocity)

def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()