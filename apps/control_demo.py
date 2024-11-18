import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import threading

class ControlNode(Node):
    dt = 0.005 # Control loop period
    target_x = 0.0
    target_y = 0.0
    target_z = 0.5  # Default height

    def __init__(self):
        super().__init__('control_node')
        self.publisher = self.create_publisher(Float32MultiArray, '/control_cmd', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

        # Start user input thread
        self.input_thread = threading.Thread(target=self.get_user_input, daemon=True)
        self.input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            try:
                user_input = input("Please enter target height (meters, current target height: {:.2f}): ".format(self.target_z))
                self.target_z = float(user_input)
                self.get_logger().info(f"Target height set to: {self.target_z} meters")
            except ValueError:
                self.get_logger().warn("Invalid input, please enter a number.")
            except EOFError:
                # Handle Ctrl+D
                break

    def control_loop(self):
        msg = Float32MultiArray()
        msg.data = [self.target_x, self.target_y, self.target_z]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    control_node = ControlNode()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Shutting down control node...")
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
