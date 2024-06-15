# roprogram_termp
2024 roprogram termproject

```sh
 $ cd src
 $ git clone https://github.com/YoungWook0533/roprogram_termp.git
 $ colcon build      #ignore warnings
 $ source install/setup.bash
```

-To view overall simulation
```sh
$ ros2 launch irb120_ros2_moveit2 irb120_interface.launch.py
```
-Spawn robots
```sh
$ ros2 ros2 launch factory_amr_description multi_amr_spawn.launch.py 
```
-Multi robot navigation node
```sh
$ ros2 launch factory_amr_navigation nav2_custom.launch.py    #currently not available
```
-To view manipulator action
```sh
$ ros2 launch ros2_execution execute_controller.py
```
-Test luggage joint
```sh
$ ros2 topic pub --once /factory_amr1/factory_amr1_luggage_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['luggage_joint'], points: [{positions: [0.3], time_from_start: {sec: 3, nanosec: 0}}]}"
```