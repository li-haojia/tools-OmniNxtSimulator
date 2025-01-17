# [0] = control type (0: position, 1: velocity, 2: angular velocity)
# [1] = x, vx, omega_x
# [2] = y, vy, omega_y
# [3] = z, vz, omega_z
# [4] = yaw, yaw, throttle

ros2 topic pub /robot_0/control_cmd std_msgs/Float32MultiArray "{data: [0.0, 1.0, 1.0, 1.0, 0.0]}"
ros2 topic pub /robot_1/control_cmd std_msgs/Float32MultiArray "{data: [0.0, 1.0, -1.0, 1.0, 0.0]}"
ros2 topic pub /robot_2/control_cmd std_msgs/Float32MultiArray "{data: [0.0, -1.0, 1.0, 1.0, 0.0]}"
ros2 topic pub /robot_3/control_cmd std_msgs/Float32MultiArray "{data: [0.0, -1.0, -1.0, 1.0, 0.0]}"