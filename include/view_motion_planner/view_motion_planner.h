#pragma once

#include <ros/ros.h>
#include <random>
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
                    const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator=false);

  MoveGroupInterface::Plan getNextPlan();

  void poseVisualizeThread();

  void graphVisualizeThread();

  void graphBuilderThread();

  octomap::point3d_collection computeVpRaycastEndpoints(const octomap::pose6d &vp);

  void computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &free, octomap::KeySet &occ, octomap::KeySet &unknown);

  void evaluateTrajectoryUtility(const robot_trajectory::RobotTrajectoryPtr &traj, const octomap::KeySet &previouslyObserved, const double &previousUtility,
                                 octomap::KeySet &observed, double &utility);

  void pathSearcherThread();

  void pathExecuterThead();

  void generateViewposeGraph();

  void plannerLoop();

  bool plannerLoopOnce(); // returns true if moved

private:
  std::default_random_engine random_engine;

  std::shared_ptr<RobotManager> robot_manager;
  std::shared_ptr<OctreeManager> octree_manager;
  ViewposeGraphManager graph_manager;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr vt_robot_state;
  rviz_visual_tools::RvizVisualToolsPtr vt_graph;

  std::vector<Viewpose> observationPoses;
  boost::shared_mutex observationPoseMtx;

};

} // namespace view_motion_planner
