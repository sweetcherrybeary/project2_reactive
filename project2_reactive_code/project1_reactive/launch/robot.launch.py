"""
Launch the reactive controller on the PHYSICAL TurtleBot 4 (Project 2).

Unlike project1.launch.py there is no simulator: the robot's own computer
already runs the drivers (lidar, Create 3 base, robot_state_publisher).
This file only starts our nodes on the lab machine, with the robot
parameters from config/robot_params.yaml and the real clock.

Keyboard control still runs in a separate terminal (see README.md).

For example:

  ros2 launch project1_reactive robot.launch.py
  ros2 launch project1_reactive robot.launch.py namespace:=/tb4_3 use_mapper:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('project1_reactive')

    default_params = os.path.join(
        pkg, 'config', 'robot_params.yaml'
    )

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    use_bringup = LaunchConfiguration('use_bringup')
    use_teleop = LaunchConfiguration('use_teleop')
    use_mapper = LaunchConfiguration('use_mapper')

    # Preserve existing TF remappings.
    tf_remaps = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # 1. Optional physical robot bringup.
    # Only enable this when running on the Raspberry Pi
    # and when the robot's bringup is not already active.
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_bringup'),
                'launch',
                'robot.launch.py'
            )
        ),
        condition=IfCondition(use_bringup),
    )

    # 2. Reactive controller.
    reactive_controller = Node(
        package='project1_reactive',
        executable='reactive_controller',
        name='reactive_controller',
        namespace=namespace,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ],
        remappings=tf_remaps,
    )

    # 3. Optional occupancy mapper.
    occupancy_mapper = Node(
        package='project1_reactive',
        executable='occupancy_mapper',
        name='occupancy_mapper',
        namespace=namespace,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ],
        remappings=tf_remaps,
        condition=IfCondition(use_mapper),
    )

    # 4. Optional keyboard teleoperation.
    # Publish to key_vel instead of cmd_vel so the
    # reactive controller handles teleop priority.
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        namespace=namespace,
        output='screen',
        prefix='xterm -e',
        parameters=[
            {'stamped': True}
        ],
        remappings=[
            ('cmd_vel', 'key_vel')
        ],
        condition=IfCondition(use_teleop),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot ROS namespace'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Physical robot parameter file'
        ),

        DeclareLaunchArgument(
            'use_bringup',
            default_value='false',
            choices=['true', 'false'],
            description='Start hardware bringup on the Raspberry Pi'
        ),

        DeclareLaunchArgument(
            'use_teleop',
            default_value='false',
            choices=['true', 'false'],
            description='Start keyboard teleoperation in xterm'
        ),

        DeclareLaunchArgument(
            'use_mapper',
            default_value='false',
            choices=['true', 'false'],
            description='Start occupancy-grid mapper'
        ),

        robot_bringup,
        reactive_controller,
        occupancy_mapper,
        keyboard_teleop,
    ])

