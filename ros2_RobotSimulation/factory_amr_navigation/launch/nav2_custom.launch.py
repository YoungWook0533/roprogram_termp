import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = True

    controller_yaml_amr1 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr1_controller.yaml')
    bt_navigator_yaml_amr1 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr1_bt_navigator.yaml')
    planner_yaml_amr1 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr1_planner_server.yaml')
    behavior_yaml_amr1 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr1_behavior.yaml')
    amr1_config = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr1_amcl_config.yaml')
    rviz_config_dir1 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'rviz', 'amr1_navigation2.rviz')

    controller_yaml_amr2 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr2_controller.yaml')
    bt_navigator_yaml_amr2 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr2_bt_navigator.yaml')
    planner_yaml_amr2 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr2_planner_server.yaml')
    behavior_yaml_amr2 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr2_behavior.yaml')
    amr2_config = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr2_amcl_config.yaml')
    rviz_config_dir2 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'rviz', 'amr2_navigation2.rviz')

    controller_yaml_amr3 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr3_controller.yaml')
    bt_navigator_yaml_amr3 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr3_bt_navigator.yaml')
    planner_yaml_amr3 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr3_planner_server.yaml')
    behavior_yaml_amr3 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr3_behavior.yaml')
    amr3_config = os.path.join(get_package_share_directory('factory_amr_navigation'), 'param', 'amr3_amcl_config.yaml')
    rviz_config_dir3 = os.path.join(get_package_share_directory('factory_amr_navigation'), 'rviz', 'amr3_navigation2.rviz')

    map_file = os.path.join(get_package_share_directory('factory_amr_navigation'), 'map', 'factory_map_saved.yaml')   
    
    return LaunchDescription([     

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]),

        # Nodes for amr1

        Node(
            namespace='factory_amr1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amr1_config]),

        Node(
            namespace='factory_amr1',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir1,],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),                                
    
        Node(
            namespace='factory_amr1',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_amr1]),

        Node(
            namespace='factory_amr1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_amr1]),
            
        Node(
            namespace='factory_amr1',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml_amr1],
            output='screen'),

        Node(
            namespace='factory_amr1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_amr1]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='factory_amr1_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        'factory_amr1/planner_server',
                                        'factory_amr1/controller_server',
                                        'factory_amr1/behavior_server',
                                        'factory_amr1/bt_navigator'
                                        ]}]),
        
        # Nodes for amr2
        
        Node(
            namespace='factory_amr2',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amr2_config]),

        Node(
            namespace='factory_amr2',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir2,],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),        

        Node(
            namespace='factory_amr2',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_amr2]),

        Node(
            namespace='factory_amr2',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_amr2]),
            
        Node(
            namespace='factory_amr2',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml_amr2],
            output='screen'),

        Node(
            namespace='factory_amr2',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_amr2]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='factory_amr2_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        'factory_amr2/planner_server',
                                        'factory_amr2/controller_server',
                                        'factory_amr2/behavior_server',
                                        'factory_amr2/bt_navigator'
                                        ]}]),

        # Nodes for amr3
        
        Node(
            namespace='factory_amr3',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amr3_config]),

        Node(
            namespace='factory_amr3',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir3,],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),        

        Node(
            namespace='factory_amr3',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_amr3]),

        Node(
            namespace='factory_amr3',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_amr3]),
            
        Node(
            namespace='factory_amr3',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml_amr3],
            output='screen'),

        Node(
            namespace='factory_amr3',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_amr3]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='factory_amr3_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        'factory_amr3/planner_server',
                                        'factory_amr3/controller_server',
                                        'factory_amr3/behavior_server',
                                        'factory_amr3/bt_navigator'
                                        ]}]),

        # Node for lifecycle_manager     
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'factory_amr1/amcl', 'factory_amr2/amcl', 'factory_amr3/amcl']}]
        )

    ])
