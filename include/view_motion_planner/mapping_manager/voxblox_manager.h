#ifndef __VOXBLOX_MAPPING_MANAGER__
#define __VOXBLOX_MAPPING_MANAGER__

#include <view_motion_planner/mapping_manager/base_mapping_manager.h>


namespace view_motion_planner
{

class VoxbloxManager: public BaseMappingManager
{
private:

public:

  VoxbloxManager(ros::NodeHandle nh, std::string frame_id);

  // Pure abstract methods from BaseMappingManager
  std::string saveToFile(const std::string &name, bool name_is_prefix);
  int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset = geometry_msgs::Transform());
  void resetMap();
  void publishMap();
  BaseMappingKeyPtr coordToKey(const octomath::Vector3 &coord);
  bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform* const sensor_origin = nullptr);

  // ;_;
  void waitForPointcloudWithRoi();
  std::vector<ViewposePtr> sampleObservationPoses(double sensorRange);
  void updateRoiTargets();
  void updateExplTargets();

};



} // namespace view_motion_planner

#endif // __VOXBLOX_MAPPING_MANAGER__