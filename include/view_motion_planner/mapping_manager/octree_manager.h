#ifndef __OCTREE_MAPPING_MANAGER__
#define __OCTREE_MAPPING_MANAGER__

#include <view_motion_planner/mapping_manager/base_mapping_manager.h>
#include <view_motion_planner/mapping_manager/octree_mapping_key.h>
#include <octomap_vpp/RoiOcTree.h>

namespace view_motion_planner
{

class OctreeManager: public BaseMappingManager
{
private:
  std::string frame_id;
  ros::NodeHandle nh;
  double resolution;

  // TODO
  boost::mutex tree_mtx;
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;

public:
  OctreeManager(ros::NodeHandle nh, std::string frame_id);

  // Pure abstract methods from BaseMappingManager
  BaseMappingKeySetPtr createEmptyKeySet();
  BaseMappingKeyRayPtr createEmptyKeyRay();
  std::string saveToFile(const std::string &name, bool name_is_prefix);
  std::string saveToFile(const std::string &name, bool name_is_prefix);
  int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset = geometry_msgs::Transform());
  void resetMap();
  void publishMap();
  BaseMappingKeyPtr coordToKey(const octomath::Vector3 &coord);
  bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform* const sensor_origin = nullptr);
  pointcloud_roi_msgs::PointcloudWithRoiPtr waitForPointcloudWithRoi(bool register_pointcloud);





  std::vector<ViewposePtr> sampleObservationPoses(double sensorRange);
  void updateRoiTargets();
  void updateExplTargets();


  // TODO
  BaseMappingKeyRayPtr computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end);


  bool computeRayCells(const octomap::point3d& origin, const octomap::point3d& end, BaseMappingKeySetPtr &freeCells, BaseMappingKeySetPtr &occCells, BaseMappingKeySetPtr &unkCells);
  void computePoseObservedCells(const octomap::pose6d &pose, BaseMappingKeySetPtr &freeCells, BaseMappingKeySetPtr &occCells, BaseMappingKeySetPtr &unkCells);

};



} // namespace view_motion_planner

#endif // __OCTREE_MAPPING_MANAGER__