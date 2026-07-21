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

// Freshie TODO: straight-line default, replace with a real algorithm.
std::vector<geometry_msgs::msg::PoseStamped> CustomPlanner::computePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  auto node = node_.lock();
  rclcpp::Time stamp = node ? node->now() : rclcpp::Clock().now();

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double total_distance = std::hypot(dx, dy);
  int n_steps = std::max(1, static_cast<int>(total_distance / interpolation_resolution_));

  bool crosses_obstacle = false;

  for (int i = 0; i <= n_steps; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(n_steps);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = goal.pose.orientation;

    unsigned int mx, my;
    if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      if (costmap_->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        crosses_obstacle = true;
      }
    }

    poses.push_back(pose);
  }

  if (crosses_obstacle) {
    RCLCPP_WARN(
      rclcpp::get_logger("CustomPlanner"),
      "Straight-line default path crosses a lethal obstacle cell -- this is "
      "expected on world2_house. Replace computePlan() with a real "
      "obstacle-avoiding algorithm.");
  }

  return poses;
}

}  // namespace planner_cpp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner_cpp::CustomPlanner, nav2_core::GlobalPlanner)
