// Copyright 2026 RoboCup Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "planner_cpp/custom_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

// Extra libraries for A* implementation
#include <algorithm>
#include <limits>
#include <queue>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace planner_cpp
{

// Read params and cache the costmap/frame this plugin will plan against.
void CustomPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  name_ = name;
  tf_ = tf;

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  RCLCPP_INFO(
    node->get_logger(), "Configured CustomPlanner \"%s\" (frame: %s)",
    name_.c_str(), global_frame_.c_str());
}

// Lifecycle no-op (nothing to release).
void CustomPlanner::cleanup()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CustomPlanner"),
    "Cleaning up plugin %s of type CustomPlanner", name_.c_str());
}

// Lifecycle no-op (nothing to start).
void CustomPlanner::activate()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CustomPlanner"),
    "Activating plugin %s of type CustomPlanner", name_.c_str());
}

// Lifecycle no-op (nothing to stop).
void CustomPlanner::deactivate()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CustomPlanner"),
    "Deactivating plugin %s of type CustomPlanner", name_.c_str());
}

// nav2_core::GlobalPlanner entry point: validates frames, then delegates to computePlan().
nav_msgs::msg::Path CustomPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  auto node = node_.lock();
  global_path.header.stamp = node ? node->now() : rclcpp::Clock().now();
  global_path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CustomPlanner"),
      "Planner will only accept start poses in the %s frame, got %s",
      global_frame_.c_str(), start.header.frame_id.c_str());
    return global_path;
  }
  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CustomPlanner"),
      "Planner will only accept goal poses in the %s frame, got %s",
      global_frame_.c_str(), goal.header.frame_id.c_str());
    return global_path;
  }

  global_path.poses = computePlan(start, goal);
  return global_path;
}

// Octile distance heuristic for A* (diagonal movement allowed).
double CustomPlanner::heuristics(int current_x, int current_y, int next_x, int next_y) const
{
  double dx = std::abs(next_x - current_x);
  double dy = std::abs(next_y - current_y);
  double h = std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
  return h;
}

// Combines the cost-so-far (g) and the heuristic (h) into the A* priority f = g + h.
double CustomPlanner::stepCost(double h, double g) const
{
  double f = g + h;
  return f;
}

namespace
{

// One entry in the A* open list. x/y are carried alongside the flat index so
// popping a node gives its coordinates directly, with no arithmetic to unpack.
struct AStarNode
{
  int idx;      // flat index, used to look up g / parent / closed
  int x, y;     // grid coordinates
  double g;     // cost from start to here
  double f;     // g + heuristic
};

// std::priority_queue is a MAX-heap: it pops whatever this ranks highest.
// Returning "a > b" therefore gives us smallest-f-first.
struct AStarCompare
{
  bool operator()(const AStarNode & a, const AStarNode & b) const {return a.f > b.f;}
};

}  // namespace

// A* over the costmap grid, 8-connected with corner-cutting disallowed.
std::vector<geometry_msgs::msg::PoseStamped> CustomPlanner::computePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  auto node = node_.lock();
  rclcpp::Time stamp = node ? node->now() : rclcpp::Clock().now();
  auto logger = rclcpp::get_logger("CustomPlanner");

  // Check for edge cases ----------------------------------------------------------------------------------------
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();

  // Check if the costmap has zero size
  if (size_x == 0 || size_y == 0) {
    RCLCPP_WARN(logger, "Costmap has zero size; returning empty path.");
    return poses;
  }

  // Convert real cordinates to grid coordinates
  unsigned int start_x, start_y, goal_x, goal_y;

  // Check if the start and goal poses are within the costmap bounds
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    RCLCPP_WARN(logger, "Start pose is outside the costmap; no path.");
    return poses;
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    RCLCPP_WARN(logger, "Goal pose is outside the costmap; no path.");
    return poses;
  }

  // A cell is a valid endpoint if it isn't inflated/lethal/unknown.
  auto isValidEndpoint = [](unsigned char cost) {
    return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };

  // Check if the start and goal cells are valid (not lethal or unknown)
  if (!isValidEndpoint(costmap_->getCost(start_x, start_y))) {
    RCLCPP_WARN(logger, "Start cell is lethal or unknown; no path.");
    return poses;
  }
  if (!isValidEndpoint(costmap_->getCost(goal_x, goal_y))) {
    RCLCPP_WARN(logger, "Goal cell is lethal or unknown; no path.");
    return poses;
  }

  const unsigned int start_index = start_y * size_x + start_x;
  const unsigned int goal_index = goal_y * size_x + goal_x;

  // Check if the start and goal are the same cell
  if (start_index == goal_index) {
    geometry_msgs::msg::PoseStamped only;
    only.header.stamp = stamp;
    only.header.frame_id = global_frame_;
    only.pose = goal.pose;
    poses.push_back(only);
    return poses;
  }

  // ---- A* search ------------------------------------------------------------------------------------------------
  // Grid coordinates need to be signed from here on: a neighbour one cell off
  // the west/north edge is -1, and that must compare correctly against 0
  // rather than wrapping around as it would if cast to unsigned first.
  const int size_x_i = static_cast<int>(size_x);
  const int size_y_i = static_cast<int>(size_y);
  const int start_xi = static_cast<int>(start_x);
  const int start_yi = static_cast<int>(start_y);
  const int goal_xi = static_cast<int>(goal_x);
  const int goal_yi = static_cast<int>(goal_y);
  const int start_idx = static_cast<int>(start_index);
  const int goal_idx = static_cast<int>(goal_index);

  // Can the robot drive on this cell? (bounds check, then cost check)
  auto traversable = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= size_x_i || y >= size_y_i) {return false;}
    return isValidEndpoint(
      costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)));
  };

  // Search state, rebuilt from scratch on every call so nothing leaks in
  // from the robot's previous position.
  const int n = size_x_i * size_y_i;
  std::vector<double> g(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);      // -1 also marks "start" and stops the backtrace
  std::vector<bool> closed(n, false);

  std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare> open;

  g[start_idx] = 0.0;
  open.push(
    {start_idx, start_xi, start_yi, 0.0,
      stepCost(heuristics(start_xi, start_yi, goal_xi, goal_yi), 0.0)});

  // 8 neighbours: 4 cardinal, then 4 diagonal
  const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;

  while (!open.empty()) {
    const AStarNode cur = open.top();   // copy: pop() invalidates the reference top() returns
    open.pop();

    // Lazy deletion: priority_queue can't update an entry in place, so a
    // better route to a cell gets pushed as a second entry. The better one
    // surfaces first; when the older one finally pops, its cell is already closed.
    if (closed[cur.idx]) {continue;}
    closed[cur.idx] = true;

    if (cur.idx == goal_idx) {found = true; break;}

    for (int i = 0; i < 8; ++i) {
      const int nx = cur.x + dx8[i];
      const int ny = cur.y + dy8[i];

      if (!traversable(nx, ny)) {continue;}

      const int n_idx = ny * size_x_i + nx;
      if (closed[n_idx]) {continue;}

      const bool diagonal = (dx8[i] != 0 && dy8[i] != 0);

      // Don't cut between two diagonally-touching walls. Legal on a grid,
      // impossible for a robot with width.
      if (diagonal &&
        (!traversable(cur.x + dx8[i], cur.y) || !traversable(cur.x, cur.y + dy8[i])))
      {
        continue;
      }

      // Step cost is pure geometry: 1 for straight, sqrt(2) for diagonal.
      const double tentative_g = cur.g + (diagonal ? std::sqrt(2.0) : 1.0);

      // Strictly better route found? Record it and queue the cell.
      if (tentative_g < g[n_idx]) {
        g[n_idx] = tentative_g;
        parent[n_idx] = cur.idx;
        const double h = heuristics(nx, ny, goal_xi, goal_yi);
        open.push({n_idx, nx, ny, tentative_g, stepCost(h, tentative_g)});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger, "No path found; open list exhausted.");
    return poses;
  }

  // ---- Walk the parent pointers back from the goal to the start -------------------------------------------------
  // The search wandered all over the map. This chain does not: every link is
  // one legal step, so the result is connected and loop-free by construction.
  std::vector<int> cells;
  for (int k = goal_idx; k != -1; k = parent[k]) {
    cells.push_back(k);
  }
  std::reverse(cells.begin(), cells.end());   // goal->start becomes start->goal

  // ---- Grid cells -> world poses ---------------------------------------------------------------------------------
  for (size_t i = 0; i < cells.size(); ++i) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(cells[i] % size_x_i),
      static_cast<unsigned int>(cells[i] / size_x_i), wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;

    if (i + 1 < cells.size()) {
      // Heading = direction to the next waypoint. Do NOT copy the goal's
      // orientation onto every pose; the controller reads these.
      double nwx, nwy;
      costmap_->mapToWorld(
        static_cast<unsigned int>(cells[i + 1] % size_x_i),
        static_cast<unsigned int>(cells[i + 1] / size_x_i), nwx, nwy);
      const double yaw = std::atan2(nwy - wy, nwx - wx);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }

    poses.push_back(pose);
  }

  return poses;
}

}  // namespace planner_cpp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner_cpp::CustomPlanner, nav2_core::GlobalPlanner)
