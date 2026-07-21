"""Top-level entry point: Gazebo + spawn, localization, navigation (branches on planner), RViz."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Mirrors robot_start_pose in src/tools/config/*.yaml; kept in sync by hand
# since installed launch files can't read src/tools/ at runtime.
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
    """Bring up Gazebo, localization, navigation (branched on planner), and RViz."""
    bringup_pkg = get_package_share_directory('bringup')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    map_yaml_path = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'maps', [world, '.yaml']])
    params_default = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'params', ['nav2_params_', model, '_default.yaml']])
    params_custom_cpp = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'params', ['nav2_params_', model, '_custom_cpp.yaml']])
    params_custom_python = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'params', ['nav2_params_', model, '_custom_python.yaml']])

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='world1_arena',
        description='Which world to load: world1_arena or world2_house')

    declare_planner_cmd = DeclareLaunchArgument(
        'planner', default_value='default',
        description='Which global planner to use: default, cpp_custom, or python_custom')

    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=EnvironmentVariable('TURTLEBOT3_MODEL', default_value='burger'),
        description='TurtleBot3 model: burger, waffle, or waffle_pi')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to start RViz')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='false',
        description='If true, skip gzclient (no GUI) -- useful for automated benchmark runs')

    # 1. Gazebo + robot spawn (planner-independent)
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
            'headless': headless,
        }.items())

    # 2. Localization -- map_server + AMCL, always stock, planner-independent
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': params_default,
            'autostart': 'true',
            'use_composition': 'False',
        }.items())

    # 3. Navigation -- branches on `planner`
    navigation_default_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        condition=LaunchConfigurationEquals('planner', 'default'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_default,
            'autostart': 'true',
            'use_composition': 'False',
        }.items())

    navigation_cpp_custom_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        condition=LaunchConfigurationEquals('planner', 'cpp_custom'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_custom_cpp,
            'autostart': 'true',
            'use_composition': 'False',
        }.items())

    navigation_python_custom_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')),
        condition=LaunchConfigurationEquals('planner', 'python_custom'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_custom_python,
            'autostart': 'true',
        }.items())

    # 4. RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    ld.add_action(gazebo_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_default_cmd)
    ld.add_action(navigation_cpp_custom_cmd)
    ld.add_action(navigation_python_custom_cmd)
    ld.add_action(rviz_cmd)
    return ld
