"""Starts Gazebo and spawns a TurtleBot3 into one of the bringup worlds; same across all modes."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start gzserver/gzclient, robot_state_publisher, and spawn the robot."""
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_path = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'worlds', [world, '.world']])

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='world1_arena',
        description='Which bringup world to load (world1_arena/world2_house)')

    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=EnvironmentVariable('TURTLEBOT3_MODEL', default_value='burger'),
        description='TurtleBot3 model: burger, waffle, or waffle_pi')

    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Initial yaw to spawn the robot at, in radians')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='false',
        description='If true, skip gzclient (no GUI) -- useful for automated benchmark runs')

    # stock turtlebot3_gazebo launch files read TURTLEBOT3_MODEL from the env,
    # not a launch arg
    set_tb3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', model)

    # scoped so gzserver.launch.py's own 'world' arg doesn't clobber ours
    gzserver_cmd = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_path}.items()),
    ])

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('headless')))

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # not stock spawn_turtlebot3.launch.py: it has no yaw arg, so we spawn
    # directly to pass -Y yaw_pose
    urdf_path = PathJoinSubstitution(
        [tb3_gazebo_share, 'models', ['turtlebot3_', model], 'model.sdf'])
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw_pose,
        ],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_yaw_pose_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(set_tb3_model_env)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    return ld
