#ifndef __BASE_MAPPING_MANAGER__
#define __BASE_MAPPING_MANAGER__

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <octomap/octomap_types.h> // octomap::point3d, octomap::pose6d
#include <view_motion_planner/mapping_manager/mapping_key.h>
#include <view_motion_planner/mapping_manager/mapping_node.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

// NOTE: Keep this file as simple as possible to prevent tangled implementations

namespace rvp_evaluation
{
class OctreeProviderInterface;
}

namespace view_motion_planner
{

enum NeighborConnectivity
{
  kSix = 6,
  kEighteen = 18,
  kTwentySix = 26
};

typedef std::vector<octomap::point3d> TargetV;
typedef std::shared_ptr<TargetV> TargetVPtr;
typedef std::shared_ptr<const TargetV> TargetVConstPtr;

class BaseMappingManager {
protected:
  ros::NodeHandle &nh;
  ros::NodeHandle &priv_nh;

  const std::string map_frame;
  const std::string ws_frame;
  tf2_ros::Buffer &tfBuffer;

  ros::Publisher roi_targets_pub;
  ros::Publisher expl_targets_pub;
  ros::Publisher border_targets_pub;

public:
//////////////////////////////////////// MAP ACCESS/MODIFICATION //////////////////////////////////////////////

  /// @brief Constructor
  /// @param nh NodeHandle
  /// @param priv_nh Private NodeHandle
  /// @param map_frame The frame_id of the map
  /// @param ws_frame The frame_id of the workspace
  /// @param tfBuffer tf2_ros::Buffer for tf lookups
  BaseMappingManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, const std::string& ws_frame, tf2_ros::Buffer &tfBuffer)
    : nh(nh), priv_nh(priv_nh), map_frame(map_frame), ws_frame(ws_frame), tfBuffer(tfBuffer)
  {
    roi_targets_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("roi_targets", 1);
    expl_targets_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("expl_targets", 1);
    border_targets_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("border_targets", 1);
  }

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
  virtual bool computeNeighborKeys(const MappingKey& point, MappingKeySet& set, const NeighborConnectivity connectivity) = 0;

  /// @brief 
  /// @param key 
  /// @return 
  virtual MappingNode search(const MappingKey& key) = 0;

  /// @brief Updates roi, expl, and border targets. These targets can be get via getRoiTargets(), getExplTargets(), and getBorderTargets()
  /// @param sr_min Sampling region min in the mapping frame
  /// @param sr_max Sampling region max in the mapping frame
  /// @param can_skip_roi roi targets may not be updated if can_skip_roi is set to true.
  /// @param can_skip_expl expl targets may not be updated if can_skip_expl is set to true.
  /// @param can_skip_border border targets may not be updated if can_skip_border is set to true.
  virtual void updateTargets(octomap::point3d sr_min, octomap::point3d sr_max, bool can_skip_roi = false, bool can_skip_expl = false, bool can_skip_border = false, NeighborConnectivity neighbor_con = kEighteen) = 0;

  /// @brief Returns ROI targets in the mapping frame. ROI targets are the unknown cells which are neighbor to a free and a ROI cell.
  /// @return Returns last computed ROI targets. Call updateTargets() beforehand to get the latest state.
  virtual TargetVConstPtr getRoiTargets() = 0;

  /// @brief Returns Expl targets in the mapping frame. Expl targets are the free cells wich are neighbor to a unknown and an occupied cell.
  /// @return Returns last computed Expl targets. Call updateTargets() beforehand to get the latest state.
  virtual TargetVConstPtr getExplTargets() = 0;

  /// @brief Returns Border targets in the mapping frame. Border targets are the free cells wich are neighbor to a unknown and a not occupied (free or unknown) cell.
  /// @return Returns last computed Border targets. Call updateTargets() beforehand to get the latest state.
  virtual TargetVConstPtr getBorderTargets() = 0;

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

  virtual void updateMap(double max_wait = 2.0) {};

  void computePoseObservedCells(const octomap::pose6d &pose, MappingKeySet &freeCells, MappingKeySet &occCells, MappingKeySet &unkCells);

  bool computeRayNodes(const octomap::point3d& origin, const octomap::point3d& end, std::vector<MappingNode> &nodes);

  /// @brief Returns a map provider that can be used for evaluation.
  /// @return If implemented, returns a map provider. Otherwise, returns nullptr.
  virtual std::shared_ptr<rvp_evaluation::OctreeProviderInterface> getMapProvider() { return nullptr; }

};


static inline std::string getMapType(std::string filepath)
{
  // TODO
  return std::string("not implemented");
}

} // namespace view_motion_planner

#endif // __BASE_MAPPING_MANAGER__