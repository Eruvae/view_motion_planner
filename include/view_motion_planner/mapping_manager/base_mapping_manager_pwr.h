#pragma once

#include "base_mapping_manager.h"

#include <pointcloud_roi_msgs/PointcloudWithRoi.h>

namespace view_motion_planner
{

// Mapping manager with PointcloudWithRoi as input
class BaseMappingManagerPwr : public BaseMappingManager
{
public:
  /// @brief Constructor
  /// @param nh NodeHandle
  /// @param priv_nh Private NodeHandle
  /// @param map_frame The frame_id of the map
  /// @param ws_frame The frame_id of the workspace
  /// @param tfBuffer tf2_ros::Buffer for tf lookups
  BaseMappingManagerPwr(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, const std::string& ws_frame, tf2_ros::Buffer &tfBuffer)
    : BaseMappingManager(nh, priv_nh, map_frame, ws_frame, tfBuffer)
  {}

  /// @brief Registers pointcloud with roi to the current map
  /// @param msg pointcloud with roi
  /// @param pc_transform If the transformation is not identity, then this transform is applied to the pointcloud before registering to the map.
  /// @return True, on success. False, on failure (e.g. pointcloud is outside of the map, no points given, tf error, etc.)
  virtual bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform) = 0;

  virtual void updateMap(double max_wait = 2.0) override;
};

} // namespace view_motion_planner