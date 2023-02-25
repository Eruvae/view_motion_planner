//////////////////////////////////////
#include <view_motion_planner/mapping_manager/octree_manager.h>
//////////////////////////////////////
#include <view_motion_planner/vmp_utils.h> // moveOctomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace view_motion_planner
{


OctreeManager::OctreeManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, double resolution): nh(nh), priv_nh(priv_nh), map_frame(map_frame), resolution(resolution)
{
  planning_tree.reset(new octomap_vpp::RoiOcTree(resolution));
  octomap_pub = priv_nh.advertise<octomap_msgs::Octomap>("octomap", 1);
}


void OctreeManager::resetMap()
{
  planning_tree->clear();
  planning_tree->clearRoiKeys();
  publishMap(); // for clearing the map on rviz
}


MappingKey OctreeManager::coordToKey(const octomap::point3d &coord)
{
  octomap::OcTreeKey key = planning_tree->coordToKey(coord);
  return toMappingKey(key);
}


bool OctreeManager::computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, MappingKeyRay& ray)
{
  tree_mtx.lock();
  octomap::KeyRay octomap_ray;
  bool ret = planning_tree->computeRayKeys(origin, end, octomap_ray);
  tree_mtx.unlock();

  if (!ret) return false;

  // Append found keys to the given ray
  MappingKeyRay mapping_key_ray = toMappingKeyRay(octomap_ray);
  ray.insert(ray.end(), mapping_key_ray.begin(), mapping_key_ray.end());

  return true;
}


bool OctreeManager::computeNeighborKeys(const MappingKey& point, const NeighborConnectivity connectivity, MappingKeySet& set)
{
  // TODO: add key limit checking?
  for (int i = 0; i < (int)connectivity; i++)
  {
    //octomap::OcTreeKey neighbour_key(point.x + octomap_vpp::nbLut[i][0], point.y + octomap_vpp::nbLut[i][1], point.z + octomap_vpp::nbLut[i][2]);
    //set.insert(toMappingKey(neighbour_key));
    MappingKey neighbour_key(point.x + octomap_vpp::nbLut[i][0], point.y + octomap_vpp::nbLut[i][1], point.z + octomap_vpp::nbLut[i][2]);
    set.insert(neighbour_key);
  }
  return true;
}


MappingNode OctreeManager::search(const MappingKey& key)
{
  tree_mtx.lock();
  octomap::OcTreeKey key_ = toOcTreeKey(key);
  octomap_vpp::RoiOcTreeNode* node_ = planning_tree->search(key_);

  if (node_ == NULL)
  {
    tree_mtx.unlock();
    MappingNode node;
    node.state = UNKNOWN;
    return node;
  }

  MappingNode node;
  node.occ_p = node_->getOccupancy();
  node.roi_p = node_->getRoiProb();

  if (node.occ_p < 0.5)
  {
    node.state = FREE;
  }
  else if (node.roi_p < 0.5)
  {
    node.state = NON_ROI;
  }
  else //if (node.roi_p >= 0.5)
  {
    node.state = ROI;
  }

  tree_mtx.unlock();
  return node;
}


bool OctreeManager::registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform)
{
  Eigen::Translation3d translation_(pc_transform.translation.x, pc_transform.translation.y, pc_transform.translation.z);
  Eigen::Quaterniond rotation_(pc_transform.rotation.w, pc_transform.rotation.x, pc_transform.rotation.y, pc_transform.rotation.z);
  bool apply_tf = !( translation_.translation().isZero() && rotation_.matrix().isIdentity() );

  octomap::Pointcloud inlierCloud, outlierCloud, fullCloud;
  if (apply_tf)
    octomap_vpp::pointCloud2ToOctomapByIndices(msg->cloud, msg->roi_indices, pc_transform, inlierCloud, outlierCloud, fullCloud);
  else
    octomap_vpp::pointCloud2ToOctomapByIndices(msg->cloud, msg->roi_indices, inlierCloud, outlierCloud, fullCloud);

  tree_mtx.lock();
  ros::Time insertStartTime(ros::Time::now());
  const octomap::point3d scan_orig(pc_transform.translation.x, pc_transform.translation.y, pc_transform.translation.z);
  planning_tree->insertPointCloud(fullCloud, scan_orig);
  planning_tree->insertRegionScan(inlierCloud, outlierCloud);
  ROS_INFO_STREAM("Inserting took " << (ros::Time::now() - insertStartTime) << " s");
  ros::Time updateTargetStartTime(ros::Time::now());
  tree_mtx.unlock();
  return true;
}


bool OctreeManager::saveToFile(const std::string &filename, bool overwrite)
{
  // TODO
  if (!overwrite)
  {
    ROS_ERROR("saveToFile: not implemented. exiting the function.");
    return false;
  }

  tree_mtx.lock();
  bool result = planning_tree->write(filename);
  tree_mtx.unlock();
  return result;
}

int OctreeManager::loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset)
{
  Eigen::Translation3d translation_(offset.translation.x, offset.translation.y, offset.translation.z);
  Eigen::Quaterniond rotation_(offset.rotation.w, offset.rotation.x, offset.rotation.y, offset.rotation.z);
  bool apply_tf = !( translation_.translation().isZero() && rotation_.matrix().isIdentity() );

  octomap_vpp::RoiOcTree *map = nullptr;
  octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(filename);
  if (!tree)
    return -1;

  map = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
  if(!map)
  {
    delete tree;
    return -2;
  }

  if (apply_tf)
  {
    moveOctomap(map, offset);
  }

  tree_mtx.lock();
  planning_tree.reset(map);
  planning_tree->computeRoiKeys();
  tree_mtx.unlock();
  publishMap();
  return 0;
}


void OctreeManager::publishMap()
{
  if (octomap_pub.getNumSubscribers() <= 0) return; // skip if no one subscribed...

  // TODO: full map publishing is not efficient. maybe publish only the changes?
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(*planning_tree, map_msg);
  tree_mtx.unlock();

  if (!msg_generated)
  {
    ROS_ERROR("publishMap: Couldn't generate the map!");
    return;
  }

  octomap_pub.publish(map_msg);
}

} // namespace view_motion_planner