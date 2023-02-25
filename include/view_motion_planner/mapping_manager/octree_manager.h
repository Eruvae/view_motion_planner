#ifndef __OCTREE_MAPPING_MANAGER__
#define __OCTREE_MAPPING_MANAGER__
//////////////////////////////////////
#include "base_mapping_manager.h"
#include "octree_type_conversions.h"
//////////////////////////////////////
#include <octomap_vpp/RoiOcTree.h>
#include <ros/ros.h>

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

  boost::mutex tree_mtx;
  std::shared_ptr<octomap_vpp::RoiOcTree> planning_tree;

public:
  OctreeManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, double resolution);

  ////////////// Pure abstract methods from BaseMappingManager /////////////
  // MAP ACCESS/MODIFICATION
  void resetMap();
  MappingKey coordToKey(const octomap::point3d &coord);
  bool computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, MappingKeyRay& ray);
  bool computeNeighborKeys(const MappingKey& point, const NeighborConnectivity connectivity, MappingKeySet& set);
  MappingNode search(const MappingKey& key);
  bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform);
  // IO
  bool saveToFile(const std::string &filename, bool overwrite = false);
  int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset = geometry_msgs::Transform());
  void publishMap();
  //////////////////////////////////////////////////////////////////////////
};



} // namespace view_motion_planner

#endif // __OCTREE_MAPPING_MANAGER__