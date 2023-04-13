#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/macros/class_forward.h>
#include <geometry_msgs/Pose.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap_types.h>

#include "view_motion_planner/mapping_manager/mapping_key.h"
#include "view_motion_planner/vmp_utils.h"
#include "view_motion_planner/robot_manager.h"

namespace view_motion_planner
{

MOVEIT_STRUCT_FORWARD(Trajectory);
MOVEIT_STRUCT_FORWARD(Viewpose);

enum TargetType {TARGET_ROI, TARGET_OCC, TARGET_BORDER};

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
  TargetType type;
  double accumulated_cost = 0;
  double accumulated_infogain = 0;
  double accumulated_utility = 0;
  MappingKeySet freeCells;
  MappingKeySet occCells;
  MappingKeySet unkCells;
  ViewposePtr pred = nullptr;
  TrajectoryPtr pred_edge = nullptr;
  size_t target_verts_on_path = 0;
  size_t total_verts_on_path = 0;
  bool visited = false;
  octomap::point3d origin;
  octomath::Vector3 dir_vec;
  double vp_dissimilarity_index;
  double vp_dissimilarity_distance;
  double vp_dissimilarity_angle;

  void addPredecessor(ViewposePtr pred, TrajectoryPtr pred_edge)
  {
    this->pred = pred;
    this->pred_edge = pred_edge;

    freeCells = pred->freeCells;
    occCells = pred->occCells;
    unkCells = pred->unkCells;

    target_verts_on_path = pred->target_verts_on_path + (!visited && (type == TARGET_ROI));
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

    accumulated_cost = 0;
    accumulated_infogain = 0;
    accumulated_utility = 0;

    target_verts_on_path = 0;
    total_verts_on_path = 0;
  }

  void computeUtility()
  {
    if (config.ig_type == Vmp_UNK_CELLS_WEIGHTED_TARGET)
      accumulated_infogain = static_cast<double>(unkCells.size()) * static_cast<double>(target_verts_on_path + 1) / static_cast<double>(total_verts_on_path + 1);
    else
      accumulated_infogain = static_cast<double>(unkCells.size());

    accumulated_utility = accumulated_cost > 0 ? accumulated_infogain / accumulated_cost : 0;
  }

  double computeViewpointDissimilarity(const ViewposePtr vp_other)
  {
      this->vp_dissimilarity_distance = 1.0;
      this->vp_dissimilarity_angle = 1.0;

      double cosine_similarity = vp_other->dir_vec.dot(this->dir_vec);
      this->vp_dissimilarity_angle = fmin(1- cosine_similarity, 2.0);
      if(this->vp_dissimilarity_angle > 1.0)
      {
          return 1.0;
      } 
      this->vp_dissimilarity_distance = fmin((vp_other->origin - this->origin).norm()/config.vpd_dist_scaling, 2.0);
      if (this->vp_dissimilarity_distance > 1.0)
      {
          return 1.0; //(this.vp_dissimilarity_distance; 
      }
      this->vp_dissimilarity_index = fmin(this->vp_dissimilarity_distance*this->vp_dissimilarity_angle, 1.0);
      return this->vp_dissimilarity_index;
  }

  bool isSimilarToPastViewpoints(const std::deque<ViewposePtr>& vp_vec, size_t num_vp = 500)
  {
      num_vp = std::min(vp_vec.size(), num_vp);
      this->vp_dissimilarity_index = 1.0;
      for(size_t i = 0; i < num_vp; ++i)
      {
          ViewposePtr past_vp = vp_vec[i];
          this->vp_dissimilarity_index = fmin(this->computeViewpointDissimilarity(past_vp), this->vp_dissimilarity_index);
          if(this->vp_dissimilarity_index < config.vpd_threshold)
          {
              ROS_DEBUG_STREAM("\t VP similar to past viewpoints: " << this->vp_dissimilarity_index << " nearness index: " << this->vp_dissimilarity_distance << " cosine distance: " << this->vp_dissimilarity_angle);
              return true;
          }
      }
      return false;
  }

};


static inline robot_trajectory::RobotTrajectoryPtr getTrajectoryForState(TrajectoryPtr t, ViewposePtr v)
{
  if (!t || !v || !t->traj) return nullptr;
  if (v->state == t->traj_start_state) return t->traj;
  if (v->state == t->bw_traj_start_state) return t->bw_traj;
  return nullptr;
}

static inline ViewposePtr sampleRandomViewPose(TargetType type, 
                                               std::shared_ptr<BaseMappingManager> mapping_manager,
                                               const std::string& map_frame, 
                                               const std::string& ws_frame,
                                               const std::string& pose_frame,
                                               tf2_ros::Buffer& tfBuffer,
                                               std::shared_ptr<RobotManager>& robot_manager)
{
  octomap::point3d origin;
  bool sample_target_success = false;
  if (type == TARGET_ROI)
  {
    auto roi_targets = mapping_manager->getRoiTargets();
    sample_target_success = getRandomTarget(*roi_targets, origin);
    if (!sample_target_success)
      type = TARGET_OCC;
  }
  if (type == TARGET_OCC)
  {
    auto expl_targets = mapping_manager->getExplTargets();
    sample_target_success = getRandomTarget(*expl_targets, origin);
    if (!sample_target_success)
      type = TARGET_BORDER;
  }
  if (type == TARGET_BORDER)
  {
    auto border_targets = mapping_manager->getBorderTargets();
    sample_target_success = getRandomTarget(*border_targets, origin);
  }
  if (!sample_target_success)
  {
    return nullptr;
  }

  octomap::point3d end;
  if (config.vp_select_type == Vmp_RANGE)
  {
    bool found_vp = false;
    for (size_t i=0; i < 100; i++) 
    {
      end = sampleRandomViewpoint(origin, config.sensor_min_range, config.sensor_max_range, global_random_engine); // TODO: using global random engine
      if (isInWorkspace(transform(end, map_frame, ws_frame, tfBuffer))) // Point in workspace region
      {
        found_vp = true;
        break;
      }
    }
    if (!found_vp)
    {
      return nullptr;
    }
  }
  else
  {
    bool found_vp = false;
    for (size_t i=0; i < 100; i++)
    {
      octomap::point3d end_ws = sampleRandomWorkspacePoint(); // uses min/max coordinates of workspace
      end = transform(end_ws, ws_frame, map_frame, tfBuffer);
      if (config.vp_select_type == Vmp_WORKSPACE_RANGE)
      {
        double dist = end.distance(origin);
        if (dist < config.sensor_min_range || dist > config.sensor_max_range)
          continue;
      }
      //if (isInWorkspace(end_ws)) // Point in workspace region
      //{
      found_vp = true;
      break;
      //}
    }
    if (!found_vp)
    {
      return nullptr;
    }
  }

  MappingKeyRay ray;
  mapping_manager->computeRayKeys(origin, end, ray);

  auto rayIt = ray.begin();
  for (auto rayEnd = ray.end(); rayIt != rayEnd; rayIt++)
  {
    MappingNode node = mapping_manager->search(*rayIt);
    if (node.isValid() && node.occ_p > 0.5) // TODO: hardcoded
    {
      return nullptr;
    } 
  }

  ViewposePtr vp(new Viewpose());
  vp->pose.position = octomap::pointOctomapToMsg(end);
  vp->pose.orientation = tf2::toMsg(getQuatInDir((origin - end).normalize()));
  vp->origin = origin; 
  vp->dir_vec = (end - origin).normalize();

  /*if(vp->isSimilarToPastViewpoints(past_viewposes_))  // old: isViewpointSimilarToPastViewpoints(past_viewposes_, vp)
  {
    return nullptr;
  }*/

  vp->state = robot_manager->getPoseRobotState(transform(vp->pose, map_frame, pose_frame, tfBuffer));
  vp->type = type;

  if (vp->state == nullptr)
  {
    return nullptr;
  }
  return vp;
}

} // namespace view_motion_planner
