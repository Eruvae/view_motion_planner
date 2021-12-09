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
    : random_engine(std::random_device{}()),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(map_frame, "/vmp_visual_markers")),
      robot_manager(new RobotManager(nh, tfBuffer, map_frame)),
      octree_manager(new OctreeManager(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, robot_manager, initialize_evaluator))
  {}

  MoveGroupInterface::Plan getNextPlan();

  void poseVisualizeThread();

  void plannerLoop();

  bool plannerLoopOnce(); // returns true if moved

private:
  std::default_random_engine random_engine;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::shared_ptr<RobotManager> robot_manager;
  std::shared_ptr<OctreeManager> octree_manager;

  std::vector<Viewpose> observationPoses;
  boost::shared_mutex observationPoseMtx;

};

} // namespace view_motion_planner
