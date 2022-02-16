#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/macros/class_forward.h>
#include <geometry_msgs/Pose.h>
#include <octomap/OcTreeKey.h>

namespace view_motion_planner
{

MOVEIT_STRUCT_FORWARD(Trajectory);
MOVEIT_STRUCT_FORWARD(Viewpose);

struct Trajectory
{
  robot_trajectory::RobotTrajectoryPtr traj, bw_traj;
  double cost;
  size_t last_collision_check_start_vertex; // Start vertex when last collision check was performed
};

struct Viewpose
{
  moveit::core::RobotStatePtr state;
  geometry_msgs::Pose pose;
  bool is_roi_targeted;
  double accumulated_cost = 0;
  double accumulated_infogain = 0;
  double accumulated_utility = 0;
  octomap::KeySet freeCells;
  octomap::KeySet occCells;
  octomap::KeySet unkCells;
  ViewposePtr pred = nullptr;
  TrajectoryPtr pred_edge = nullptr;

  void addPredecessor(ViewposePtr pred, TrajectoryPtr pred_edge)
  {
    this->pred = pred;
    this->pred_edge = pred_edge;

    freeCells = pred->freeCells;
    occCells = pred->occCells;
    unkCells = pred->unkCells;

    this->accumulated_cost = pred->accumulated_cost + pred_edge->cost;
  }

  void clearPredecessor()
  {
    this->pred = nullptr;
    this->pred_edge = nullptr;

    freeCells.clear();
    occCells.clear();
    unkCells.clear();
  }

  void computeUtility()
  {
    accumulated_infogain = unkCells.size();
    accumulated_utility = accumulated_cost > 0 ? accumulated_infogain / accumulated_cost : 0;
  }
};

} // namespace view_motion_planner
