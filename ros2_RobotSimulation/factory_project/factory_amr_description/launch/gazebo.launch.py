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

    amr2_share_dir = get_package_share_directory('factory_amr_description')
    amr2_xacro_file = os.path.join(amr2_share_dir, 'urdf', 'robot_urdf_2.xacro')
    amr2_robot_description_config = xacro.process_file(amr2_xacro_file)
    amr2_robot_urdf = amr2_robot_description_config.toxml()
    amr2_robot_controllers = os.path.join(amr2_share_dir, 'config', 'controller_factory_amr2.yaml')
    
    amr3_share_dir = get_package_share_directory('factory_amr_description')
    amr3_xacro_file = os.path.join(amr3_share_dir, 'urdf', 'robot_urdf_3.xacro')
    amr3_robot_description_config = xacro.process_file(amr3_xacro_file)
    amr3_robot_urdf = amr3_robot_description_config.toxml()
    amr3_robot_controllers = os.path.join(amr3_share_dir, 'config', 'controller_factory_amr3.yaml')

    amr2_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='factory_amr2',
        name='robot_state_publisher',
        parameters=[{'frame_prefix': 'factory_amr2/',
                    'use_sim_time': True,
                    'robot_description': amr2_robot_urdf}],
        remappings=[('/scan', '/factory_amr2/scan')]
    )

    amr2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='factory_amr2',
        parameters=[amr2_robot_controllers],
        output='both',
    )

    amr2_urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='factory_amr2',
        arguments=[
            '-entity', 'factory_amr2',
            '-topic', 'robot_description',
            '-x', '-0.2',
            '-y', '-0.8',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',   
            '-Y', '4.71',  
            '-robot_namespace', 'factory_amr2',
        ],
        output='screen'
    )

    amr2_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='factory_amr2',
        arguments=['factory_amr2_joint_state_broadcaster', '--controller-manager', '/factory_amr2/controller_manager'],
        output='screen'
    )

    amr2_luggage_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='factory_amr2',
        arguments=['factory_amr2_luggage_controller', '--controller-manager', '/factory_amr2/controller_manager'],
        output='screen'
    )

    amr2_delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=amr2_urdf_spawn_node,
            on_exit=[amr2_joint_state_broadcaster_spawner]
        )
    )

    amr2_delay_luggage_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=amr2_joint_state_broadcaster_spawner,
            on_exit=[amr2_luggage_controller_spawner]
        )
    )

    amr3_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='factory_amr3',
        name='robot_state_publisher',
        parameters=[{'frame_prefix': 'factory_amr3/',
                    'use_sim_time': True,
                    'robot_description': amr3_robot_urdf}],
        remappings=[('/scan', '/factory_amr3/scan')]
    )

    amr3_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='factory_amr3',
        parameters=[amr3_robot_controllers],
        output='both',
    )

    amr3_urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='factory_amr3',
        arguments=[
            '-entity', 'factory_amr3',
            '-topic', 'robot_description',
            '-x', '1.1',
            '-y', '-0.2',
            '-z', '0.0',
            '-R', '0.0',   
            '-P', '0.0',   
            '-Y', '0.0',  
            '-robot_namespace', 'factory_amr3',
        ],
        output='screen'
    )

    amr3_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='factory_amr3',
        arguments=['factory_amr3_joint_state_broadcaster', '--controller-manager', '/factory_amr3/controller_manager'],
        output='screen'
    )

    amr3_luggage_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='factory_amr3',
        arguments=['factory_amr3_luggage_controller', '--controller-manager', '/factory_amr3/controller_manager'],
        output='screen'
    )

    # DECLARE Gazebo WORLD file:
    irb120_ros2_gazebo = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'),
        'worlds',
        'irb120.world'
    )
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': irb120_ros2_gazebo}.items(),
    )

    amr3_delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=amr3_urdf_spawn_node,
            on_exit=[amr3_joint_state_broadcaster_spawner]
        )
    )

    amr3_delay_luggage_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=amr3_joint_state_broadcaster_spawner,
            on_exit=[amr3_luggage_controller_spawner]
        )
    )

    amr3_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=amr2_luggage_controller_spawner,
            on_exit=[
                amr3_robot_state_publisher_node,
                amr3_control_node,
                amr3_urdf_spawn_node,
                amr3_delay_joint_state_broadcaster_spawner_after_spawn_entity,
                amr3_delay_luggage_controller_spawner_after_joint_state_broadcaster_spawner,
            ]
        )
    )

    return LaunchDescription([
        gazebo,



        amr3_robot_state_publisher_node,
        amr3_control_node,
        amr3_urdf_spawn_node,
        amr3_delay_joint_state_broadcaster_spawner_after_spawn_entity,
        amr3_delay_luggage_controller_spawner_after_joint_state_broadcaster_spawner,
        #amr3_event_handler,
    ])

#ros2 topic pub --once /amr_luggage_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['luggage_joint'], points: [{positions: [1.0], time_from_start: {sec: 3, nanosec: 0}}]}"
#ros2 topic pub /factory_amr/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0,nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"