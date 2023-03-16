//////////////////////////////////////
#include <view_motion_planner/mapping_manager/voxblox_manager.h>
//////////////////////////////////////
//#include <view_motion_planner/vmp_utils.h> // TODO
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
#include <filesystem>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ros_params.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl_ros/transforms.h>


namespace view_motion_planner
{


VoxbloxManager::VoxbloxManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, double resolution): nh(nh), priv_nh(priv_nh), map_frame(map_frame), resolution(resolution), inv_resolution(1.0/resolution)
{
  voxblox::TsdfMap::Config config = voxblox::getTsdfMapConfigFromRosParam(priv_nh);
  voxblox::TsdfIntegratorBase::Config integrator_config = voxblox::getTsdfIntegratorConfigFromRosParam(priv_nh);
  voxblox::MeshIntegratorConfig mesh_config = voxblox::getMeshIntegratorConfigFromRosParam(priv_nh);

  // overwrite protected members
  config.tsdf_voxel_size = 0.004;
  config.tsdf_voxels_per_side = 64;
  integrator_config.voxel_carving_enabled = true;

  ROS_WARN("================ voxblox::TsdfMap::Config ================");
  ROS_INFO("tsdf_voxel_size: %f", config.tsdf_voxel_size);
  ROS_INFO("tsdf_voxels_per_side: %ld", config.tsdf_voxels_per_side);
  ROS_WARN("================ voxblox::TsdfIntegratorBase::Config ================");
  ROS_INFO("allow_clear: %d", integrator_config.allow_clear);
  ROS_INFO("clear_checks_every_n_frames: %d", integrator_config.clear_checks_every_n_frames);
  ROS_INFO("default_truncation_distance: %f", integrator_config.default_truncation_distance);
  ROS_INFO("enable_anti_grazing: %d", integrator_config.enable_anti_grazing);
  ROS_INFO("integration_order_mode: %s", integrator_config.integration_order_mode.c_str());
  ROS_INFO("integrator_threads: %d", integrator_config.integrator_threads);
  ROS_INFO("max_consecutive_ray_collisions: %d", integrator_config.max_consecutive_ray_collisions);
  ROS_INFO("max_integration_time_s: %f", integrator_config.max_integration_time_s);
  ROS_INFO("max_ray_length_m: %f", integrator_config.max_ray_length_m);
  ROS_INFO("max_weight: %f", integrator_config.max_weight);
  ROS_INFO("min_ray_length_m: %f", integrator_config.min_ray_length_m);
  ROS_INFO("sparsity_compensation_factor: %f", integrator_config.sparsity_compensation_factor);
  ROS_INFO("start_voxel_subsampling_factor: %f", integrator_config.start_voxel_subsampling_factor);
  ROS_INFO("use_const_weight: %d", integrator_config.use_const_weight);
  ROS_INFO("use_sparsity_compensation_factor: %d", integrator_config.use_sparsity_compensation_factor);
  ROS_INFO("use_weight_dropoff: %d", integrator_config.use_weight_dropoff);
  ROS_INFO("voxel_carving_enabled: %d", integrator_config.voxel_carving_enabled);
  ROS_WARN("================ voxblox::MeshIntegratorConfig ================");
  ROS_INFO("integrator_threads: %d", mesh_config.integrator_threads);
  ROS_INFO("min_weight: %f", mesh_config.min_weight);
  ROS_INFO("use_color: %d", mesh_config.use_color);
  ROS_WARN("====================================================");

  tsdf_mutex.lock();
  tsdf_server.reset(new voxblox::ModifiedTsdfServer(nh, priv_nh, config, integrator_config, mesh_config, map_frame));
  tsdf_mutex.unlock();
}


void 
VoxbloxManager::resetMap()
{
  tsdf_mutex.lock();
  tsdf_server->clear();
  tsdf_mutex.unlock();
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
  // tree_mtx.lock();
  // octomap::OcTreeKey key_ = toOcTreeKey(key);
  // octomap_vpp::RoiOcTreeNode* node_ = planning_tree->search(key_);

  // if (node_ == NULL)
  // {
  //   tree_mtx.unlock();
  //   MappingNode node;
  //   node.state = UNKNOWN;
  //   return node;
  // }

  MappingNode node;
  // node.occ_p = node_->getOccupancy();
  // node.roi_p = node_->getRoiProb();

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

  //tree_mtx.unlock();
  return node;
}


bool 
VoxbloxManager::registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform)
{
  Eigen::Translation3d translation_(pc_transform.translation.x, pc_transform.translation.y, pc_transform.translation.z);
  Eigen::Quaterniond rotation_(pc_transform.rotation.w, pc_transform.rotation.x, pc_transform.rotation.y, pc_transform.rotation.z);
  Eigen::Isometry3d pc_inv_transformd = (translation_ * rotation_).inverse();
  Eigen::MatrixXf pc_inv_transformf = pc_inv_transformd.matrix().cast<float>();

  // TODO: pc already comes transformed. but we need the previous state. unfortunately pc_transform is empty here.
  ROS_FATAL_STREAM(pc_inv_transformf);
  
  // TODO: inverse transform pointcloud! a terrible workaround to adapt voxblox
  std::unique_ptr<sensor_msgs::PointCloud2> inv_msg(new sensor_msgs::PointCloud2());
  pcl_ros::transformPointCloud(pc_inv_transformf, msg->cloud, *inv_msg);

  ROS_FATAL("frame_id: %s", msg->cloud.header.frame_id.c_str());

  // Preprocess pointcloud and set PointXYZI where roi indices have high intensity values.
  // TODO: Doesn't work with PointXYZI. Worked with PointXYZRGB.
  pcl::PointCloud<pcl::PointXYZRGB> pc_xyzi;
  pcl::fromROSMsg(*inv_msg, pc_xyzi); // TODO: This line is printing "Failed to find match for field 'intensity'." 
  for (int i=0; i<pc_xyzi.size(); i++)
  {
    pc_xyzi[i].a = 255;
    pc_xyzi[i].r = 0;
    pc_xyzi[i].g = 0;
    pc_xyzi[i].b = 0;
  }
  for (int i=0; i<msg->roi_indices.size(); i++)
  {
    int idx = msg->roi_indices[i];
    if (idx < 0 || idx >= pc_xyzi.size())
    {
      ROS_FATAL("registerPointcloudWithRoi: There is a problem with roi_indices in pointcloud_roi!!!");
      return false;
    }
    pc_xyzi[idx].a = 255;
    pc_xyzi[idx].r = 255;
    pc_xyzi[idx].g = 255;
    pc_xyzi[idx].b = 255;
  }
  sensor_msgs::PointCloud2 pc2_xyzi;
  pcl::toROSMsg(pc_xyzi, pc2_xyzi);

  // Insert pointcloud
  tsdf_mutex.lock();
  ros::Time insert_time_start(ros::Time::now());
  voxblox::Transformation voxtransform;
  tf::transformMsgToKindr(pc_transform, &voxtransform);
  tsdf_server->processPointCloudMessageAndInsert(pc2_xyzi, voxtransform, /*is_freespace_pointcloud=*/false);
  ROS_INFO_STREAM("Inserting took " << (ros::Time::now() - insert_time_start) << " s");
  tsdf_mutex.unlock();

  tsdf_initialized = true;
  return true;
}


void 
VoxbloxManager::updateTargets(octomap::point3d sr_min, octomap::point3d sr_max, bool can_skip_roi, bool can_skip_expl, bool can_skip_border, NeighborConnectivity neighbor_con)
{
  std::scoped_lock lock(tsdf_mutex, targets_mtx); // more robust to deadlocks

  for (unsigned int i = 0; i < 3; i++)
  {
    if (sr_min(i) > sr_max(i))
      std::swap(sr_min(i), sr_max(i));
  }

  // TODO: TEMPORARLY
  roi_targets = std::make_shared<TargetV>();
  expl_targets = std::make_shared<TargetV>();
  border_targets = std::make_shared<TargetV>();

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

  tsdf_mutex.lock();
  bool result = tsdf_server->saveMap(filename);
  tsdf_mutex.unlock();

  return result;
}

int 
VoxbloxManager::loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset)
{
  Eigen::Translation3d translation_(offset.translation.x, offset.translation.y, offset.translation.z);
  Eigen::Quaterniond rotation_(offset.rotation.w, offset.rotation.x, offset.rotation.y, offset.rotation.z);
  bool apply_tf = !( translation_.translation().isZero() && rotation_.matrix().isIdentity() );

  if (apply_tf)
  {
    ROS_FATAL("VoxbloxManager::loadFromFile: offset is not supported!");
  }

  tsdf_mutex.lock();
  bool result = tsdf_server->loadMap(filename);
  tsdf_mutex.unlock();

  if (!result) return -1;
  return 0;
}


void 
VoxbloxManager::publishMap()
{
  if (!tsdf_initialized) return;
  tsdf_mutex.lock();
  tsdf_server->updateMesh(); // publishes pointclouds, tsdf, and mesh
  //tsdf_server->generateMesh(); // publishes pointclouds, tsdf, and mesh
  //tsdf_server->publishPointclouds();
  tsdf_mutex.unlock();
}

} // namespace view_motion_planner