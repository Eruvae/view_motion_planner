#ifndef __BASE_MAPPING_MANAGER__
#define __BASE_MAPPING_MANAGER__

#include <string>
#include <ros/ros.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>
#include <geometry_msgs/Transform.h>
#include <octomap/octomap_types.h> // octomap::point3d, octomap::pose6d
#include <view_motion_planner/mapping_manager/mapping_key.h>
#include <view_motion_planner/mapping_manager/mapping_node.h>

// NOTE: Keep this file as simple as possible to prevent tangled implementations

namespace view_motion_planner
{

enum NeighborConnectivity
{
  kSix = 6,
  kEighteen = 18,
  kTwentySix = 26
};

class BaseMappingManager {
public:
//////////////////////////////////////// MAP ACCESS/MODIFICATION //////////////////////////////////////////////

  /// @brief Resets the mapping information. Other information (e.g. frame_id) remains the same.
  virtual void resetMap() = 0;

  /// @brief Converts floating point coordinates to key values which can be used for indexing.
  /// @param coord Position w.r.t. the frame_id.
  /// @return The key which can be used for indexing.
  virtual MappingKey coordToKey(const octomap::point3d &coord) = 0;

  /// @brief Find keys along the ray
  /// @param origin 
  /// @param end 
  /// @param ray 
  /// @return (not decided)
  virtual bool computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, MappingKeyRay& ray) = 0;

  /// @brief 
  /// @param point 
  /// @param connectivity 
  /// @param set 
  /// @return (not decided)
  virtual bool computeNeighborKeys(const MappingKey& point, const NeighborConnectivity connectivity, MappingKeySet& set) = 0;

  /// @brief 
  /// @param key 
  /// @return 
  virtual MappingNode search(const MappingKey& key) = 0;

  /// @brief Registers pointcloud with roi to the current map
  /// @param msg pointcloud with roi
  /// @param pc_transform If the transformation is not identity, then this transform is applied to the pointcloud before registering to the map.
  /// @return True, on success. False, on failure (e.g. pointcloud is outside of the map, no points given, tf error, etc.)
  virtual bool registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform) = 0;

//////////////////////////////////////// IO //////////////////////////////////////////////

  /// @brief Serializes 3D map to a file.
  /// @param filename Full path to a file.
  /// @param overwrite If false, overwrites if a file with the same filename exists.
  /// @return Returns true on success, returns false otherwise.
  virtual bool saveToFile(const std::string &filename, bool overwrite = false) = 0;

  /// @brief Deserializes 3D map from a file
  /// @param filename Full path to a file.
  /// @param offset 
  /// @return 0 (success), -1 (failed to deserialize), -2 (wrong map type)
  virtual int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset = geometry_msgs::Transform()) = 0;

  /// @brief Publishes the map for visualization (e.g. on RViz).
  ///        This can be octree, mesh, pointcloud, primitive lists, etc. 
  //         The method is depends on the map type and the results should be used for debugging/visualization purposes.
  //         Published topic must be in the private namespace.
  virtual void publishMap() = 0;

};


static inline std::string getMapType(std::string filepath)
{
  // TODO
  return std::string("not implemented");
}

} // namespace view_motion_planner

#endif // __BASE_MAPPING_MANAGER__