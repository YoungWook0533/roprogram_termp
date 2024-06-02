#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: July, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

# irb120_simulation.launch.py:
# Launch file for the ABB-IRB120 Robot GAZEBO SIMULATION in ROS2 Foxy:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # *********************** AMR *********************** #
    amr1_share_dir = get_package_share_directory('factory_amr_description')
    amr1_xacro_file = os.path.join(amr1_share_dir, 'urdf', 'robot_urdf_1.xacro')
    amr1_robot_description_config = xacro.process_file(amr1_xacro_file, mappings={'robot_namespace': 'factory_amr1'})
    amr1_robot_urdf = amr1_robot_description_config.toxml()

    amr1_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='factory_amr1',
        name='robot_state_publisher',
        parameters=[{'frame_prefix': 'factory_amr1/',
                    'use_sim_time': True,
                    'robot_description': amr1_robot_urdf}]
    )

    amr1_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='factory_amr1',
        name='joint_state_publisher'
    )

    amr1_urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='factory_amr1',
        arguments=[
            '-entity', 'factory_amr1',
            '-topic', 'robot_description',
            '-x', '0.4',
            '-y', '0.75',
            '-z', '0.0',
            '-R', '0.0',   
            '-P', '0.0',   
            '-Y', '1.57',  
            '-robot_namespace', 'factory_amr1',
        ],
        output='screen'
    )
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    irb120_ros2_gazebo = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'),
        'worlds',
        'irb120.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': irb120_ros2_gazebo}.items(),
             )
    

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo, 
        amr1_urdf_spawn_node,               
        amr1_robot_state_publisher_node,    
        amr1_joint_state_publisher_node,    
    ])