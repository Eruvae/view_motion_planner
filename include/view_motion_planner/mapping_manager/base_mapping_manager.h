#ifndef __BASE_MAPPING_MANAGER__
#define __BASE_MAPPING_MANAGER__

#include <string>
#include <ros/ros.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>
#include <geometry_msgs/Transform.h>
#include <octomap/octomap_types.h> // octomap::point3d, octomap::pose6d

#include <view_motion_planner/viewpose.h> // TODO: this must be removed, I think
#include <view_motion_planner/mapping_manager/base_mapping_key.h>

// Keep this file as simple as possible to prevent tangled implementations

namespace view_motion_planner
{

class BaseMappingManager {
public:

  /// @brief Creates an empty KeySet for the related mapping type
  /// @return Pointer to the KeySet
  virtual BaseMappingKeySetPtr createEmptyKeySet() = 0;

  /// @brief Creates an empty KeyRay for the related mapping type
  /// @return Pointer to the KeyRay
  virtual BaseMappingKeyRayPtr createEmptyKeyRay() = 0;

  /// @brief Serializes 3D map to a file
  /// @param name Full path excluding the file extension
  /// @param name_is_prefix If true, adds a timestamp between the save path and the file extension
  /// @return Returns the full path on success. Returns empty string on failure.
  virtual std::string saveToFile(const std::string &name, bool name_is_prefix) = 0;

  /// @brief Deserializes 3D map from a file
  /// @param filename
  /// @param offset 
  /// @return 0 (success), -1 (failed to deserialize), -2 (wrong map type)
  virtual int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset = geometry_msgs::Transform()) = 0;

  /// @brief Resets the mapping information. Other information (e.g. frame_id) remains the same.
  virtual void resetMap() = 0;

  /// @brief Publishes the map for visualization (e.g. on RViz).
  virtual void publishMap() = 0;

  /// @brief Converts floating point coordinates to key values which can be used for indexing.
  /// @param coord Position w.r.t. the frame_id.
  /// @return The key which can be used for indexing.
  virtual BaseMappingKeyPtr coordToKey(const octomap::point3d &coord) = 0;

  /// @brief Registers pointcloud with roi to the current map
  /// @param msg pointcloud with roi
  /// @param sensor_origin If set, transforms the given pointcloud accordingly. Uses tf2 and msg.header.frame_id if not set.
  /// @return True, on success. False, on failure (e.g. pointcloud is outside of the map, no points given, tf error, etc.)
  virtual bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform* const sensor_origin = nullptr) = 0;

  /// @brief Subscribes to the input topic and waits for single a pointcloud with roi message using ros::topic::waitForMessage.
  /// @param register_pointcloud If true, calls registerPointcloudWithRoi internally after receiving the message.
  /// @return Returns pointcloud with roi message on success. Returns nullptr on failure.
  virtual pointcloud_roi_msgs::PointcloudWithRoiPtr waitForPointcloudWithRoi(bool register_pointcloud) = 0;

  /// @brief 
  /// @param pose 
  /// @param freeCells 
  /// @param occCells 
  /// @param unkCells 
  virtual void computePoseObservedCells(const octomap::pose6d &pose, BaseMappingKeySetPtr &freeCells, BaseMappingKeySetPtr &occCells, BaseMappingKeySetPtr &unkCells) = 0;

  /// @brief 
  /// @param origin 
  /// @param end 
  /// @return 
  virtual BaseMappingKeyRayPtr computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end);


  // TODO
  
  virtual void updateRoiTargets() = 0;
  virtual void updateExplTargets() = 0;

  // TODO: utilize this method to remove viewpose dependency
  //virtual std::vector<ViewposePtr> sampleObservationPoses(double sensorRange) = 0;

  // TODO: only used internally. maybe utilize these methods
  //virtual bool computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, octomap::KeyRay& ray) = 0;
  //virtual bool computeRayCells(const octomap::point3d& origin, const octomap::point3d& end, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells) = 0;



  // neighbor keys?

};

// TODO: implement a function here to get the map type from file


} // namespace view_motion_planner

#endif // __BASE_MAPPING_MANAGER__