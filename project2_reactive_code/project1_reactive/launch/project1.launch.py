"""
Launch the Project 1 simulation.

Starts, in order:
  1. Gazebo with the Project 1 world (worlds/project1.sdf),
  2. the simulated TurtleBot 4 (spawned with its dock) and its ROS bridges,
  3. our reactive_controller and occupancy_mapper nodes,
  4. RViz showing the occupancy grid (optional).

Keyboard control runs in a separate terminal (see README.md).

The Gazebo part follows turtlebot4_gz_bringup/launch/sim.launch.py from the
TurtleBot 4 simulator (Apache 2.0, Clearpath Robotics), changed so that
Gazebo can also find the worlds in this package.

To start the robot somewhere else, pass x, y and yaw, for instance
  ros2 launch project1_reactive project1.launch.py x:=1.5 y:=3.8 yaw:=0.0
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    pkg_project1 = get_package_share_directory('project1_reactive')
    pkg_tb4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_gz_gui_plugins = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_create_description = get_package_share_directory('irobot_create_description')
    pkg_create_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')
    pkg_create_gz_plugins = get_package_share_directory('irobot_create_gz_plugins')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    params_file = os.path.join(pkg_project1, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_project1, 'rviz', 'project1.rviz')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value='project1',
            description='World file name without ".sdf"; must equal its <world name>'),
        DeclareLaunchArgument('model', default_value='standard',
                              choices=['standard', 'lite'],
                              description='TurtleBot 4 model'),
        DeclareLaunchArgument('x', default_value='1.524',
                              description='Robot start x (m); default = room center'),
        DeclareLaunchArgument('y', default_value='3.81',
                              description='Robot start y (m); default = room center'),
        DeclareLaunchArgument('z', default_value='0.0', description='Robot start z (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Robot start heading (rad); 0 = facing +x'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Open RViz with the occupancy grid'),
    ]

    # Where Gazebo looks for worlds and models: our worlds first, then the
    # TurtleBot 4 and Create 3 packages (robot meshes, dock, sample worlds).
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_project1, 'worlds'),
            os.path.join(pkg_tb4_gz_bringup, 'worlds'),
            os.path.join(pkg_create_gz_bringup, 'worlds'),
            str(Path(pkg_tb4_description).parent.resolve()),
            str(Path(pkg_create_description).parent.resolve()),
        ]))
    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_tb4_gz_gui_plugins, 'lib'),
            os.path.join(pkg_create_gz_plugins, 'lib'),
        ]))

    # Gazebo: load <world>.sdf, start running immediately (-r), and use the
    # TurtleBot 4 GUI layout.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'), '.sdf', ' -r', ' -v 2',
                ' --gui-config ',
                PathJoinSubstitution(
                    [pkg_tb4_gz_bringup, 'gui', LaunchConfiguration('model'), 'gui.config']),
            ]),
        ])

    # Simulation clock from Gazebo to ROS (all nodes use sim time).
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # The TurtleBot 4 itself: robot model, dock, sensor bridges, Create 3 nodes.
    # The bridges need the world name to find the robot's sensor topics.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_tb4_gz_bringup, 'launch',
                                   'turtlebot4_spawn.launch.py'])]),
        launch_arguments=[
            ('namespace', ''),
            ('world', LaunchConfiguration('world')),
            ('rviz', 'false'),
            ('model', LaunchConfiguration('model')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ])

    # Our nodes. Started a few seconds later so the robot exists first.
    our_nodes = TimerAction(period=5.0, actions=[
        Node(package='project1_reactive', executable='reactive_controller',
             name='reactive_controller', output='screen',
             parameters=[params_file, {'use_sim_time': True}]),
        Node(package='project1_reactive', executable='occupancy_mapper',
             name='occupancy_mapper', output='screen',
             parameters=[params_file, {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])

    return LaunchDescription(arguments + [
        gz_resource_path,
        gz_gui_plugin_path,
        gazebo,
        clock_bridge,
        robot,
        our_nodes,
    ])
