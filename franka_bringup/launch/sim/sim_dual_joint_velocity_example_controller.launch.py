#  Copyright (c) 2023 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # robot_ip_1_parameter_name = 'robot_ip_1'
    # robot_ip_2_parameter_name = 'robot_ip_2'
    arm_id_1_parameter_name = 'arm_id_1'
    arm_id_2_parameter_name = 'arm_id_2'
    load_gripper_1_parameter_name = 'load_gripper_1'
    load_gripper_2_parameter_name = 'load_gripper_2'
    # use_fake_hardware_parameter_name = 'use_fake_hardware'
    # fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    # use_rviz_parameter_name = 'use_rviz'
    # scene_xml_parameter_name = 'scene_xml'
    # mj_yaml_parameter_name = 'mj_yaml'

    # robot_ip_1 = LaunchConfiguration(robot_ip_1_parameter_name)
    # robot_ip_2 = LaunchConfiguration(robot_ip_2_parameter_name)
    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)
    load_gripper_1 = LaunchConfiguration(load_gripper_1_parameter_name)
    load_gripper_2 = LaunchConfiguration(load_gripper_2_parameter_name)
    # use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    # scene_xml = LaunchConfiguration(scene_xml_parameter_name)
    # mj_yaml = LaunchConfiguration(mj_yaml_parameter_name)


    # default_scene_xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', 'scene.xml')
    # default_mj_yaml_file = os.path.join(get_package_share_directory('franka_bringup'), 'config', 'mujoco', 'mj_objects.yaml')


    return LaunchDescription([
        # DeclareLaunchArgument(
        #     robot_ip_1_parameter_name,
        #     default_value='192.168.0.201',
        #     description='Hostname or IP address of the robot 1.'),
        # DeclareLaunchArgument(
        #     robot_ip_2_parameter_name,
        #     default_value='192.168.0.202',
        #     description='Hostname or IP address of the robot 2.'),
        # DeclareLaunchArgument(
        #     use_rviz_parameter_name,
        #     default_value='false',
        #     description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='mj_left',
            description='Unique arm ID of robot 1.'),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='mj_right',
            description='Unique arm ID of robot 2.'),
        DeclareLaunchArgument(
            load_gripper_1_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            load_gripper_2_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        # DeclareLaunchArgument(
        #     scene_xml_parameter_name,
        #     default_value=default_scene_xml_file,
        #     description='The path to the mujoco xml file that you want to load.'
        # ),
        # DeclareLaunchArgument(
        #     mj_yaml_parameter_name,
        #     default_value=default_mj_yaml_file,
        #     description='The path to the mujoco object yaml file that you want to load.'
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'sim/dual_franka_sim.launch.py'])]),
            launch_arguments={
                            #   use_rviz_parameter_name: use_rviz,
                              arm_id_1_parameter_name: arm_id_1,
                              arm_id_2_parameter_name: arm_id_2,
                              load_gripper_1_parameter_name: load_gripper_1,
                              load_gripper_2_parameter_name: load_gripper_2,
                            #   scene_xml_parameter_name: scene_xml,
                            #   mj_yaml_parameter_name: mj_yaml
                              }.items(),
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['dual_joint_velocity_example_controller'],
            output='screen',
        ),
    ])
