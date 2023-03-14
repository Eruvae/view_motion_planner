//////////////////////////////////////
#include <view_motion_planner/mapping_manager/voxblox_manager.h>
//////////////////////////////////////
//#include <view_motion_planner/vmp_utils.h> // TODO
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
#include <filesystem>

namespace view_motion_planner
{


VoxbloxManager::VoxbloxManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, double resolution): nh(nh), priv_nh(priv_nh), map_frame(map_frame), resolution(resolution), inv_resolution(1.0/resolution)
{
  //planning_tree.reset(new octomap_vpp::RoiOcTree(resolution));
  mesh_pub = priv_nh.advertise<voxblox_msgs::Mesh>("vis_voxblox_mesh", 1, true);
}


void 
VoxbloxManager::resetMap()
{
  // planning_tree->clear();
  // planning_tree->clearRoiKeys();
  // publishMap(); // for clearing the map on rviz
}

// TODO: not tested
MappingKey 
VoxbloxManager::coordToKey(const octomap::point3d &coord)
{
  voxblox::GlobalIndex vkey(coord.x()*inv_resolution, coord.y()*inv_resolution, coord.z()*inv_resolution);
  return toMappingKey(vkey);
}

// TODO: not tested
bool 
VoxbloxManager::computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, MappingKeyRay& ray)
{
  voxblox::Point start_scaled(origin.x()*inv_resolution, origin.y()*inv_resolution, origin.z()*inv_resolution);
  voxblox::Point end_scaled(end.x()*inv_resolution, end.y()*inv_resolution, end.z()*inv_resolution);
  voxblox::RayCaster ray_caster(start_scaled, end_scaled);

  bool success = false;
  voxblox::GlobalIndex ray_index;
  while (ray_caster.nextRayIndex(&ray_index))
  {
    ray.push_back(toMappingKey(ray_index));
    success = true;
  }
  return success;
}

// TODO: not tested
bool 
VoxbloxManager::computeNeighborKeys(const MappingKey& point, const NeighborConnectivity connectivity, MappingKeySet& set)
{
  voxblox::GlobalIndex vkey = toVoxbloxKey(point);

  if (connectivity == NeighborConnectivity::kSix)
  {
    voxblox::Neighborhood<voxblox::Connectivity::kSix>::IndexMatrix neighbor_indices;
    voxblox::Neighborhood<voxblox::Connectivity::kSix>::getFromGlobalIndex(vkey, &neighbor_indices);
    for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
      const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
      set.insert(toMappingKey(neighbor_index));
    }
  }
  else if (connectivity == NeighborConnectivity::kEighteen)
  {
    voxblox::Neighborhood<voxblox::Connectivity::kEighteen>::IndexMatrix neighbor_indices;
    voxblox::Neighborhood<voxblox::Connectivity::kEighteen>::getFromGlobalIndex(vkey, &neighbor_indices);
    for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
      const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
      set.insert(toMappingKey(neighbor_index));
    }
  }
  else if (connectivity == NeighborConnectivity::kTwentySix)
  {
    voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::IndexMatrix neighbor_indices;
    voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::getFromGlobalIndex(vkey, &neighbor_indices);
    for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
      const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
      set.insert(toMappingKey(neighbor_index));
    }
  }
  else
  {
    ROS_FATAL("VoxbloxManager::computeNeighborKeys: Unsupported neighbor connectivity.");
    return false;
  }

  return true;
}


MappingNode 
VoxbloxManager::search(const MappingKey& key)
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

  if (node.occ_p < 0.5) // TODO: hardcoded
  {
    node.state = FREE;
  }
  else if (node.roi_p < 0.5) // TODO: hardcoded
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


bool 
VoxbloxManager::registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform)
{
  // Eigen::Translation3d translation_(pc_transform.translation.x, pc_transform.translation.y, pc_transform.translation.z);
  // Eigen::Quaterniond rotation_(pc_transform.rotation.w, pc_transform.rotation.x, pc_transform.rotation.y, pc_transform.rotation.z);
  // bool apply_tf = !( translation_.translation().isZero() && rotation_.matrix().isIdentity() );

  // octomap::Pointcloud inlierCloud, outlierCloud, fullCloud;
  // if (apply_tf)
  //   octomap_vpp::pointCloud2ToOctomapByIndices(msg->cloud, msg->roi_indices, pc_transform, inlierCloud, outlierCloud, fullCloud);
  // else
  //   octomap_vpp::pointCloud2ToOctomapByIndices(msg->cloud, msg->roi_indices, inlierCloud, outlierCloud, fullCloud);

  // tree_mtx.lock();
  // ros::Time insert_time_start(ros::Time::now());
  // const octomap::point3d scan_orig(pc_transform.translation.x, pc_transform.translation.y, pc_transform.translation.z);

  // TODO: QUESTION: insertPointCloud and insertRegionScan???
  // planning_tree->insertPointCloud(fullCloud, scan_orig);
  // planning_tree->insertRegionScan(inlierCloud, outlierCloud);


  // ROS_INFO_STREAM("Inserting took " << (ros::Time::now() - insert_time_start) << " s");
  // tree_mtx.unlock();
  return true;
}


void 
VoxbloxManager::updateTargets(octomap::point3d sr_min, octomap::point3d sr_max, bool can_skip_roi, bool can_skip_expl, bool can_skip_border, NeighborConnectivity neighbor_con)
{
  std::scoped_lock lock(tree_mtx, targets_mtx); // more robust to deadlocks

  for (unsigned int i = 0; i < 3; i++)
  {
    if (sr_min(i) > sr_max(i))
      std::swap(sr_min(i), sr_max(i));
  }

  /*
  if (!can_skip_roi)
  {
    TargetVPtr new_roi_targets = std::make_shared<TargetV>();
    //new_roi_targets->reserve(size_t(roi_targets->size() * 1.1)); // TODO: Enable for more performance
    
    octomap::KeySet roi = planning_tree->getRoiKeys();
    octomap::KeySet freeNeighbours;
    for (const octomap::OcTreeKey &key : roi)
    {
      if (!isInSamplingRegion(planning_tree->keyToCoord(key), sr_min, sr_max)) // ROI not in sampling region
        continue;

      planning_tree->getNeighborsInState(key, freeNeighbours, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::FREE_NONROI, (octomap_vpp::Neighborhood)neighbor_con );
    }
    for (const octomap::OcTreeKey &key : freeNeighbours)
    {
      if (planning_tree->hasNeighborInState(key, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::UNKNOWN, (octomap_vpp::Neighborhood)neighbor_con ))
      {
        new_roi_targets->push_back(planning_tree->keyToCoord(key));
      }
    }
    // update roi targets
    roi_targets = new_roi_targets;
  }

  if (!can_skip_expl || !can_skip_border)
  {
    TargetVPtr new_expl_targets = std::make_shared<TargetV>();
    //new_expl_targets->reserve(size_t(expl_targets->size() * 1.1)); // TODO: Enable for more performance
    TargetVPtr new_border_targets = std::make_shared<TargetV>();
    //new_border_targets->reserve(size_t(border_targets->size() * 1.1)); // TODO: Enable for more performance

    for (auto it = planning_tree->begin_leafs_bbx(sr_min, sr_max), end = planning_tree->end_leafs_bbx(); it != end; it++)
    {
      if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
      {
        if (planning_tree->hasNeighborInState(it.getKey(), octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::UNKNOWN, octomap_vpp::NB_6))
        {
          if (planning_tree->hasNeighborInState(it.getKey(), octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::OCCUPIED_ROI, octomap_vpp::NB_6))
            new_expl_targets->push_back(it.getCoordinate());
          else
            new_border_targets->push_back(it.getCoordinate());
        }
      }
    }
    // update expl and border targets
    expl_targets = new_expl_targets;
    border_targets = new_border_targets;
  }
  */

}


TargetVConstPtr 
VoxbloxManager::getRoiTargets()
{
  targets_mtx.lock();
  TargetVConstPtr tmp = roi_targets;
  targets_mtx.unlock();
  return tmp;
}


TargetVConstPtr 
VoxbloxManager::getExplTargets()
{
  targets_mtx.lock();
  TargetVConstPtr tmp = expl_targets;
  targets_mtx.unlock();
  return tmp;
}


TargetVConstPtr
VoxbloxManager::getBorderTargets()
{
  targets_mtx.lock();
  TargetVConstPtr tmp = border_targets;
  targets_mtx.unlock();
  return tmp;
}


bool 
VoxbloxManager::saveToFile(const std::string &filename, bool overwrite)
{
  if (!overwrite)
  {
    const std::filesystem::path p = filename;
    if (std::filesystem::exists(p))
    {
      ROS_ERROR("Can't save the map. File already exists: %s", filename.c_str());
      return false;
    }
  }

  tree_mtx.lock();

  // TODO
  //bool result = planning_tree->write(filename);

  tree_mtx.unlock();
  // return result;

  return false; // TODO
}

int 
VoxbloxManager::loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset)
{
  // Eigen::Translation3d translation_(offset.translation.x, offset.translation.y, offset.translation.z);
  // Eigen::Quaterniond rotation_(offset.rotation.w, offset.rotation.x, offset.rotation.y, offset.rotation.z);
  // bool apply_tf = !( translation_.translation().isZero() && rotation_.matrix().isIdentity() );

  // octomap_vpp::RoiOcTree *map = nullptr;
  // octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(filename);
  // if (!tree)
  //   return -1;

  // map = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
  // if(!map)
  // {
  //   delete tree;
  //   return -2;
  // }

  // if (apply_tf)
  // {
  //   moveOctomap(map, offset);
  // }

  // tree_mtx.lock();
  // planning_tree.reset(map);
  // planning_tree->computeRoiKeys();
  // tree_mtx.unlock();
  // publishMap();
  return 0;
}


void 
VoxbloxManager::publishMap()
{
  if (mesh_pub.getNumSubscribers() <= 0) return; // skip if no one subscribed...

  // TODO
  // octomap_msgs::Octomap mesh_msg;
  // mesh_msg.header.frame_id = map_frame;
  // mesh_msg.header.stamp = ros::Time::now();
  // tree_mtx.lock();
  // bool msg_generated = octomap_msgs::fullMapToMsg(*planning_tree, mesh_msg);
  // tree_mtx.unlock();

  // if (!msg_generated)
  // {
  //   ROS_ERROR("publishMap: Couldn't generate the map!");
  //   return;
  // }

  // mesh_pub.publish(mesh_msg);
}

} // namespace view_motion_planner