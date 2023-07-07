#pragma once

#include "base_mapping_manager.h"
#include "voxblox_type_conversions.h"

#include <panoptic_mapping_ros/panoptic_mapper.h>

namespace view_motion_planner
{

class PanopticManager : public BaseMappingManager
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  std::string map_frame;
  double resolution;
  double inv_resolution;
  std::recursive_mutex targets_mtx; // lock before updating or exposing target shared_ptr's because shared_ptr modification is not atomic!
  TargetVPtr roi_targets;
  TargetVPtr expl_targets;
  TargetVPtr border_targets;

  std::recursive_mutex map_mutex;
  std::shared_ptr<panoptic_mapping::PanopticMapper> panoptic_mapper;

public:
  PanopticManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, double resolution);
  
  ////////////// Pure abstract methods from BaseMappingManager /////////////
  // MAP ACCESS/MODIFICATION
  void resetMap();
  MappingKey coordToKey(const octomap::point3d &coord);
  bool computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, MappingKeyRay& ray);
  bool computeNeighborKeys(const MappingKey& point, MappingKeySet& set, const NeighborConnectivity connectivity);
  MappingNode search(const MappingKey& key);
  bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform);
  void updateTargets(octomap::point3d sr_min, octomap::point3d sr_max, bool update_roi, bool update_expl, bool update_border, NeighborConnectivity neighbor_con);
  TargetVConstPtr getRoiTargets();
  TargetVConstPtr getExplTargets();
  TargetVConstPtr getBorderTargets();
  // IO
  bool saveToFile(const std::string &filename, bool overwrite);
  int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset);
  void publishMap();
  //////////////////////////////////////////////////////////////////////////
};

} // namespace view_motion_planner