"""Instructor-only: Gazebo + slam_toolbox for building a world map. See instructor.md."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# copy of bringup.launch.py's WORLD_SPAWN_POSE, kept in sync by hand
WORLD_SPAWN_POSE = {
    'world1_arena': {'x': '-2.0', 'y': '-0.5', 'yaw': str(math.radians(0))},
    'world2_house': {'x': '0.0', 'y': '0.0', 'yaw': str(math.radians(0))},
}


def launch_setup(context, *args, **kwargs):
    """Declare x_pose/y_pose/yaw_pose defaults for the selected world."""
    world_name = LaunchConfiguration('world').perform(context)
    pose = WORLD_SPAWN_POSE.get(world_name, {'x': '0.0', 'y': '0.0', 'yaw': '0.0'})
    return [
        DeclareLaunchArgument('x_pose', default_value=pose['x']),
        DeclareLaunchArgument('y_pose', default_value=pose['y']),
        DeclareLaunchArgument('yaw_pose', default_value=pose['yaw']),
    ]


def generate_launch_description():
    """Bring up Gazebo, slam_toolbox, and RViz for mapping a world."""
    bringup_pkg = get_package_share_directory('bringup')
    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')
    slam_rviz_config = os.path.join(bringup_pkg, 'rviz', 'slam_view.rviz')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='world1_arena',
        description='Which bringup world to SLAM (world1_arena/world2_house)')

    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=EnvironmentVariable('TURTLEBOT3_MODEL', default_value='burger'),
        description='TurtleBot3 model: burger, waffle, or waffle_pi')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz (with the slam_toolbox plugin panel)')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'model': model,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose,
            'use_sim_time': use_sim_time,
        }.items())

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(bringup_pkg, 'params', 'slam_toolbox_params.yaml'),
        }.items())

    # slam_toolbox has no built-in RViz launch; reuse ours with our own config
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': slam_rviz_config,
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    ld.add_action(gazebo_cmd)
    ld.add_action(slam_toolbox_cmd)
    ld.add_action(rviz_cmd)
    return ld
