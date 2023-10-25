# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_description'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("brobot_base"), "config", "ekf.yaml"]
    )

    default_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'launch', 'default_robot.launch.py']
    )

    custom_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'launch', 'custom_robot.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='custom_robot', 
            default_value='false',
            description='Use custom robot'
        ),

        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyACM0',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='joy', 
            default_value='false',
            description='Use Joystick'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_robot_launch_path),
            condition=UnlessCondition(LaunchConfiguration("custom_robot")),
            launch_arguments={
                'base_serial_port': LaunchConfiguration("base_serial_port")
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(custom_robot_launch_path),
            condition=IfCondition(LaunchConfiguration("custom_robot")),
        )
    ])
