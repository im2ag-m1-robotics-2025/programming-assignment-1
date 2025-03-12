# programming-assignment-1
To create:
ros2 pkg create my_py_pkg --build-type ament_python --dependencies
rclpy

ros2 pkg create my_robot_interfaces

ros2 pkg create my_robot_bringup --build-type ament_cmake

To run:

cd ~/ros2_ws
colcon build
source ~/.bashrc  (if you modified your bash file as in chapter 4)

ros2 launch my_robot_bringup draw_square.launch.py

ros2 service call /draw_square_service my_robot_interfaces/srv/DrawSquare "{side_length: 2.0, velocity: 1.0}"

OR

ros2 action send_goal /draw_square_action my_robot_interfaces/action/DrawSquare "{side_length: 2.0, velocity: 1.0}"

you can make the argument like "{}" because there is an default value for side length and velocity.
