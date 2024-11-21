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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_1_parameter_name = 'robot_ip_1'
    robot_ip_2_parameter_name = 'robot_ip_2'
    arm_id_1_parameter_name = 'arm_id_1'
    arm_id_2_parameter_name = 'arm_id_2'
    load_gripper_1_parameter_name = 'load_gripper_1'
    load_gripper_2_parameter_name = 'load_gripper_2'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip_1 = LaunchConfiguration(robot_ip_1_parameter_name)
    robot_ip_2 = LaunchConfiguration(robot_ip_2_parameter_name)
    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)
    load_gripper_1 = LaunchConfiguration(load_gripper_1_parameter_name)
    load_gripper_2 = LaunchConfiguration(load_gripper_2_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)


    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_1_parameter_name,
            default_value='192.168.0.201',
            description='Hostname or IP address of the robot 1.'),
        DeclareLaunchArgument(
            robot_ip_2_parameter_name,
            default_value='192.168.0.202',
            description='Hostname or IP address of the robot 2.'),
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='left',
            description='Unique arm ID of robot 1.'),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='right',
            description='Unique arm ID of robot 2.'),
        DeclareLaunchArgument(
            load_gripper_1_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            load_gripper_2_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'real/dual_franka.launch.py'])]),
            launch_arguments={robot_ip_1_parameter_name: robot_ip_1,
                              robot_ip_2_parameter_name: robot_ip_2,
                              arm_id_1_parameter_name: arm_id_1,
                              arm_id_2_parameter_name: arm_id_2,
                              load_gripper_1_parameter_name: load_gripper_1,
                              load_gripper_2_parameter_name: load_gripper_2,
                              use_fake_hardware_parameter_name: use_fake_hardware,
                              fake_sensor_commands_parameter_name: fake_sensor_commands,
                              use_rviz_parameter_name: use_rviz
                              }.items(),
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['dual_joint_velocity_example_controller'],
            output='screen',
        ),
    ])
