"""Boilerplate action server (freshies do NOT edit): drop-in replacement for planner_server."""
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from planner_py.occupancy_grid_view import OccupancyGridView
from planner_py.custom_planner import generate_path

_MAP_WAIT_TIMEOUT_SEC = 10.0
_TF_LOOKUP_TIMEOUT_SEC = 2.0


class PlanningFailed(Exception):
    """Internal signal that a leg could not be planned; caller aborts the goal."""


class ComputePathToPoseServer(Node):

    def __init__(self):
        """Set up the /map subscription, TF listener, and both action servers."""
        super().__init__('custom_planner')
        self._callback_group = ReentrantCallbackGroup()

        self._map = None
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos,
            callback_group=self._callback_group)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            execute_callback=self._execute_to_pose_callback,
            callback_group=self._callback_group,
        )
        self._through_poses_action_server = ActionServer(
            self,
            ComputePathThroughPoses,
            'compute_path_through_poses',
            execute_callback=self._execute_through_poses_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            'custom_planner ready: serving compute_path_to_pose '
            'and compute_path_through_poses')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Cache the latest /map message."""
        self._map = msg

    def _wait_for_map(self) -> bool:
        """Block until a /map message has been received or the timeout elapses."""
        deadline = time.time() + _MAP_WAIT_TIMEOUT_SEC
        while self._map is None and time.time() < deadline:
            time.sleep(0.1)
        return self._map is not None

    def _lookup_current_pose(self) -> PoseStamped:
        """Look up the robot's current pose via the map->base_link TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time(),
                timeout=rclpy.duration.Duration(seconds=_TF_LOOKUP_TIMEOUT_SEC))
        except TransformException as exc:
            self.get_logger().error(f'Could not look up map->base_link: {exc}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _plan_leg(self, start_pose: PoseStamped, goal_pose: PoseStamped, grid) -> list:
        """Run generate_path() for one leg; raise PlanningFailed on error/no path."""
        try:
            waypoints = generate_path(start_pose, goal_pose, grid)
        except Exception as exc:  # noqa: BLE001 -- a buggy student planner must not crash the node
            self.get_logger().error(f'custom_planner.generate_path() raised: {exc}')
            raise PlanningFailed from exc
        if not waypoints:
            raise PlanningFailed
        return waypoints

    def _build_path_msg(self, waypoints: list) -> Path:
        """Package (x, y) waypoints into a nav_msgs/Path."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _resolve_start_pose(self, request) -> PoseStamped:
        """Use the request's start pose if given, else look up the live pose."""
        if request.use_start:
            return request.start
        return self._lookup_current_pose()

    def _execute_to_pose_callback(self, goal_handle):
        """Serve a single compute_path_to_pose goal."""
        request = goal_handle.request

        start_pose = self._resolve_start_pose(request)
        if start_pose is None:
            goal_handle.abort()
            return ComputePathToPose.Result()

        if not self._wait_for_map():
            self.get_logger().error('No /map received -- cannot plan.')
            goal_handle.abort()
            return ComputePathToPose.Result()

        grid = OccupancyGridView(self._map)
        t0 = time.time()
        try:
            waypoints = self._plan_leg(start_pose, request.goal, grid)
        except PlanningFailed:
            self.get_logger().warn('No path found for compute_path_to_pose goal.')
            goal_handle.abort()
            return ComputePathToPose.Result()
        planning_time_sec = time.time() - t0

        goal_handle.succeed()
        result = ComputePathToPose.Result()
        result.path = self._build_path_msg(waypoints)
        result.planning_time = Duration(
            sec=int(planning_time_sec),
            nanosec=int((planning_time_sec % 1.0) * 1e9))
        return result

    def _execute_through_poses_callback(self, goal_handle):
        """Serve a compute_path_through_poses goal by planning leg by leg."""
        request = goal_handle.request

        start_pose = self._resolve_start_pose(request)
        if start_pose is None or not request.goals:
            goal_handle.abort()
            return ComputePathThroughPoses.Result()

        if not self._wait_for_map():
            self.get_logger().error('No /map received -- cannot plan.')
            goal_handle.abort()
            return ComputePathThroughPoses.Result()

        grid = OccupancyGridView(self._map)
        t0 = time.time()
        all_waypoints = []
        leg_start = start_pose
        try:
            for leg_goal in request.goals:
                leg_waypoints = self._plan_leg(leg_start, leg_goal, grid)
                if all_waypoints:  # skip duplicate junction pose
                    leg_waypoints = leg_waypoints[1:]
                all_waypoints.extend(leg_waypoints)
                leg_start = leg_goal
        except PlanningFailed:
            self.get_logger().warn('No path found for compute_path_through_poses goal.')
            goal_handle.abort()
            return ComputePathThroughPoses.Result()
        planning_time_sec = time.time() - t0

        goal_handle.succeed()
        result = ComputePathThroughPoses.Result()
        result.path = self._build_path_msg(all_waypoints)
        result.planning_time = Duration(
            sec=int(planning_time_sec),
            nanosec=int((planning_time_sec % 1.0) * 1e9))
        return result


def main(args=None):
    """Spin up ComputePathToPoseServer."""
    rclpy.init(args=args)
    node = ComputePathToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
