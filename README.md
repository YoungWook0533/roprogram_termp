# roprogram_termp
2024 roprogram termproject

$ cd src
$ git clone https://github.com/YoungWook0533/roprogram_termp.git
$ colcon build      //ignore warnings
$ source install/setup.bash

-To view overall simulation
$ ros2 launch irb120_ros2_moveit2 irb120_interface.launch.py

-Multi robot navigation node
$ ros2 launch factory_amr_navigation nav2_custom.launch.py

-To view manipulator action
$ ros2 launch ros2_execution execute_controller.py

-To see amr luggage_joint moving (dump luggages)
$ ros2 launch factory_amr_description gazebo.launch.py  //for testing luggage_joint, will be merged into irb120_interface.launch.py soon

-Test luggage joint
$ ros2 topic pub --once /amr_luggage_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['luggage_joint'], points: [{positions: [1.0], time_from_start: {sec: 3, nanosec: 0}}]}"