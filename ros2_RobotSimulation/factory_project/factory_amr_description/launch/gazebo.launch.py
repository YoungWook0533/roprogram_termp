from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('factory_amr_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot_urdf_3.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    robot_controllers = os.path.join(share_dir, 'config', 'controller_factory_amr3.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': True}
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    irb120_ros2_gazebo = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'),
        'worlds',
        'irb120.world'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': irb120_ros2_gazebo,
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'factory_amr3',
            '-topic', 'robot_description',
            '-x', '0.5',
            '-y', '0.9',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '1.57'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['amr3_joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['factory_amr3', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    luggage_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['amr3_luggage_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_joint_state_broadcaster_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_luggage_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[luggage_controller_spawner]
        )
    )

    delay_diff_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner]
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_joint_state_broadcaster_after_control_node,
        delay_luggage_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_diff_controller_spawner_after_joint_state_broadcaster_spawner,
    ])

#ros2 topic pub --once /amr_luggage_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['luggage_joint'], points: [{positions: [1.0], time_from_start: {sec: 3, nanosec: 0}}]}"
#ros2 topic pub /factory_amr/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0,nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"