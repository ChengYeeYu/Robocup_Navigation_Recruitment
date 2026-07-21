// Copyright 2026 RoboCup Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef PLANNER_CPP__CUSTOM_PLANNER_HPP_
#define PLANNER_CPP__CUSTOM_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace planner_cpp
{

// Freshie-editable global planner plugin (nav2_core::GlobalPlanner); only
// computePlan()'s body is meant to be edited.
class CustomPlanner : public nav2_core::GlobalPlanner
{
public:
  CustomPlanner() = default;
  ~CustomPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // TODO(freshie): plan start->goal using costmap_ (worldToMap/mapToWorld/
  // getCost). Return an ordered, stamped vector; empty if no path found.
  std::vector<geometry_msgs::msg::PoseStamped> computePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_, name_;
  double interpolation_resolution_;
};

}  // namespace planner_cpp

#endif  // PLANNER_CPP__CUSTOM_PLANNER_HPP_
