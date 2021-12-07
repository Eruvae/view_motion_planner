#pragma once

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include "view_motion_planner/octree_manager.h"
#include "view_motion_planner/robot_manager.h"

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class ViewMotionPlanner
{
public:
  ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                    const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator=false)
    : robot_manager(new RobotManager(nh, tfBuffer, map_frame)),
      octree_manager(new OctreeManager(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, robot_manager, initialize_evaluator))
  {}

  MoveGroupInterface::Plan getNextPlan();

  void plannerLoop();

  bool plannerLoopOnce(); // returns true if moved

private:
  std::shared_ptr<RobotManager> robot_manager;
  std::shared_ptr<OctreeManager> octree_manager;

};

} // namespace view_motion_planner
