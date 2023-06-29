import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_path = get_package_share_directory("catbot_simulation")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_description = get_package_share_directory("catbot_description")

    gazebo_params_file = os.path.join(pkg_path, 'config/gazebo_params.yaml')
    rviz_config = os.path.join(
        pkg_description,
        "rviz",
        "catbot.rviz")

    # Args
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    launch_rviz = LaunchConfiguration("launch_rviz")



    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_path, "worlds", "obstacles.world"),
        description="Gazebo world")

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='True',
        description='Use ros2_control if true')

    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether to launch rviz2")

    # Nodes
    rviz_cmd = Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([launch_rviz])),
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time,
                          'use_ros2_control': use_ros2_control}.items())

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', # noqa
                            'gazebo.launch.py')]), launch_arguments={'world': world, # noqa
                            'extra_gazebo_args': '--ros-args --params-file ' + # noqa
                            gazebo_params_file}.items()) # noqa

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'catbot',
                #    "-timeout", "120",
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1',
                   '-Y', '0.0'])
    
    # Spawn diff_controller
    diff_drive_spawner = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'])

    # Spawn joint_state_broadcaser
    joint_broad_spawner = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'])


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(use_sim_time_cmd)
    ld.add_action(use_ros2_control_cmd)
    ld.add_action(world_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)

    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    ld.add_action(launch_rviz_cmd)
    ld.add_action(rviz_cmd)

    return ld
