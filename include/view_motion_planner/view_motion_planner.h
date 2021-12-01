#pragma once

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include "view_motion_planner/octree_manager.h"

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class ViewMotionPlanner
{
public:
  ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &map_frame, double tree_resolution, bool initialize_evaluator=false)
    : octree_manager(nh, tfBuffer, map_frame, tree_resolution, initialize_evaluator)
  {}

  MoveGroupInterface::Plan getNextPlan();

private:
  OctreeManager octree_manager;

};

} // namespace view_motion_planner
