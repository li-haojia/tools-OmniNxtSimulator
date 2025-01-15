ros2 topic pub /robot_0/control_cmd std_msgs/Float32MultiArray "{data: [1.0, 1.0, 1.0]}"
ros2 topic pub /robot_1/control_cmd std_msgs/Float32MultiArray "{data: [1.0, -1.0, 1.0]}"
ros2 topic pub /robot_2/control_cmd std_msgs/Float32MultiArray "{data: [-1.0, 1.0, 1.0]}"
ros2 topic pub /robot_3/control_cmd std_msgs/Float32MultiArray "{data: [-1.0, -1.0, 1.0]}"