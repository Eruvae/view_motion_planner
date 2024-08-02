#ifndef __VOXBLOX_MAPPING_MANAGER__
#define __VOXBLOX_MAPPING_MANAGER__
//////////////////////////////////////
#include "base_mapping_manager_pwr.h"
#include "voxblox_type_conversions.h"
//////////////////////////////////////
#include <octomap_vpp/RoiOcTree.h>
#include <ros/ros.h>
#include <mutex>
#include <view_motion_planner/voxblox_tsdf_server.h> // modified header
#include <voxblox_msgs/Mesh.h>
#include <voxblox/utils/neighbor_tools.h>

namespace view_motion_planner
{

class VoxbloxManager: public BaseMappingManagerPwr
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

  std::recursive_mutex tsdf_mutex;
  std::shared_ptr<voxblox::ModifiedTsdfServer> tsdf_server;
  bool tsdf_initialized = false;

public:
  VoxbloxManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, const std::string& ws_frame, tf2_ros::Buffer &tfBuffer, double resolution);

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

#endif // __VOXBLOX_MAPPING_MANAGER__