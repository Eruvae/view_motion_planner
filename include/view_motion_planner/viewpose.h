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
  moveit::core::RobotStatePtr traj_start_state, bw_traj_start_state;
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
  size_t target_verts_on_path = 0;
  size_t total_verts_on_path = 0;

  void addPredecessor(ViewposePtr pred, TrajectoryPtr pred_edge)
  {
    this->pred = pred;
    this->pred_edge = pred_edge;

    freeCells = pred->freeCells;
    occCells = pred->occCells;
    unkCells = pred->unkCells;

    target_verts_on_path = pred->target_verts_on_path + is_roi_targeted ? 1 : 0;
    total_verts_on_path = pred->total_verts_on_path + 1;

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
    accumulated_infogain = static_cast<double>(unkCells.size());// * static_cast<double>(target_verts_on_path + 1) / static_cast<double>(total_verts_on_path + 1);
    accumulated_utility = accumulated_cost > 0 ? accumulated_infogain / accumulated_cost : 0;
  }
};

static inline robot_trajectory::RobotTrajectoryPtr getTrajectoryForState(TrajectoryPtr t, ViewposePtr v)
{
  if (!t || !v || !t->traj) return nullptr;
  if (v->state == t->traj_start_state) return t->traj;
  if (v->state == t->bw_traj_start_state) return t->bw_traj;
  return nullptr;
}

} // namespace view_motion_planner
