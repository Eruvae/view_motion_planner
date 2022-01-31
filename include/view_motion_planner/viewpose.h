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
};

struct Viewpose
{
  moveit::core::RobotStatePtr state;
  geometry_msgs::Pose pose;
  bool is_roi_targeted;
  double accumulated_cost = 0;
  double accumalated_utility = 0;
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

  void computeUtility()
  {
    if (accumulated_cost == 0)
    {
      accumalated_utility = 0;
      return;
    }
    accumalated_utility = unkCells.size() / accumulated_cost;
  }

  bool operator< (const Viewpose &rhs) const
  {
    return accumalated_utility < rhs.accumalated_utility;
  }
};

} // namespace view_motion_planner
