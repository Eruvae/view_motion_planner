#ifndef __OCTREE_MAPPING_MANAGER__
#define __OCTREE_MAPPING_MANAGER__
//////////////////////////////////////
#include "base_mapping_manager.h"
#include "octree_type_conversions.h"
//////////////////////////////////////
#include <octomap_vpp/RoiOcTree.h>
#include <ros/ros.h>
#include <mutex>

namespace view_motion_planner
{

class OctreeManager: public BaseMappingManager
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  std::string map_frame;
  double resolution;
  ros::Publisher octomap_pub;
  std::recursive_mutex targets_mtx; // lock before updating or exposing target shared_ptr's because shared_ptr modification is not atomic!
  TargetVPtr roi_targets;
  TargetVPtr expl_targets;
  TargetVPtr border_targets;

  std::recursive_mutex tree_mtx;
  std::shared_ptr<octomap_vpp::RoiOcTree> planning_tree;

public:
  OctreeManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, double resolution);

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
  std::shared_ptr<rvp_evaluation::OctreeProviderInterface> getMapProvider() override;
  // IO
  bool saveToFile(const std::string &filename, bool overwrite);
  int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset);
  void publishMap();
  //////////////////////////////////////////////////////////////////////////
};



} // namespace view_motion_planner

#endif // __OCTREE_MAPPING_MANAGER__