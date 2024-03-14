ros2 topic pub /obs_pos_vel std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 topic pub /current_step std_msgs/msg/Int16 "{data: 0}"
ros2 topic pub /mobile_desired_parameters geometry_msgs/msg/Pose "{position: {x: 170, y: 100, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0}}"
ros2 launch ur_robot_driver ur5e.launch.py ur_type:=ur5e robot_ip:=192.168.0.102 launch_rviz:=true initial_joint_controller:=forward_velocity_controller
ros2 launch dghc demo.launch.py
ros2 launch teleop_joy teleop-launch.py

