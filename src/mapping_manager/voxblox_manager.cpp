//////////////////////////////////////
#include <view_motion_planner/mapping_manager/voxblox_manager.h>
//////////////////////////////////////
#include <view_motion_planner/vmp_utils.h>
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


VoxbloxManager::VoxbloxManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, const std::string& ws_frame, tf2_ros::Buffer &tfBuffer, double resolution)
  : BaseMappingManagerPwr(nh, priv_nh, map_frame, ws_frame, tfBuffer), resolution(resolution), inv_resolution(1.0/resolution)
{
  voxblox::TsdfMap::Config config = voxblox::getTsdfMapConfigFromRosParam(priv_nh);
  voxblox::TsdfIntegratorBase::Config integrator_config = voxblox::getTsdfIntegratorConfigFromRosParam(priv_nh);
  voxblox::MeshIntegratorConfig mesh_config = voxblox::getMeshIntegratorConfigFromRosParam(priv_nh);

  // overwrite protected members
  config.tsdf_voxel_size = resolution;
  integrator_config.default_truncation_distance = config.tsdf_voxel_size*2; // TODO
  config.tsdf_voxels_per_side = 64;
  integrator_config.voxel_carving_enabled = true; // if false, from surface to truncated distance will be registered. if true, from origin to truncated distance will be registered.
  integrator_config.allow_clear = false;
  integrator_config.max_ray_length_m = 2.0; // TODO

  ROS_WARN("================ voxblox::TsdfMap::Config ================");
  ROS_INFO("tsdf_voxel_size: %f", config.tsdf_voxel_size);
  ROS_INFO("tsdf_voxels_per_side: %ld", config.tsdf_voxels_per_side);
  ROS_WARN("================ voxblox::TsdfIntegratorBase::Config ================");
  ROS_INFO("allow_clear: %d", integrator_config.allow_clear);
  ROS_INFO("clear_checks_every_n_frames: %d", integrator_config.clear_checks_every_n_frames);
  ROS_INFO("default_truncation_distance: %f", integrator_config.default_truncation_distance);
  ROS_INFO("enable_anti_grazing: %d", integrator_config.enable_anti_grazing);
  ROS_INFO("integration_order_mode: %s", integrator_config.integration_order_mode.c_str());
  ROS_INFO("integrator_threads: %lu", integrator_config.integrator_threads);
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
  ROS_INFO("integrator_threads: %lu", mesh_config.integrator_threads);
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

bool 
VoxbloxManager::computeNeighborKeys(const MappingKey& point, MappingKeySet& set, const NeighborConnectivity connectivity)
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
  const voxblox::TsdfVoxel* voxel = tsdf_server->tsdf_map_->getTsdfLayer().getVoxelPtrByGlobalIndex(toVoxbloxKey(key));
  MappingNode node;

  if (!voxel || !voxblox::utils::isObservedVoxel(*voxel))
  {
    node.state = UNKNOWN;
  }
  else if (voxel->distance > 0)
  {
    node.state = FREE;
    node.occ_p = 0;
    node.roi_p = 0;
  }
  else if (voxel->color.r > 50)
  {
    node.state = ROI;
    node.occ_p = 1;
    node.roi_p = 1;
  }
  else
  {
    node.state = NON_ROI;
    node.occ_p = 1;
    node.roi_p = 0;
  }

  return node;
}


bool 
VoxbloxManager::registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform)
{
  Eigen::Translation3d translation_(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
  Eigen::Quaterniond rotation_(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
  bool is_msg_transform_identity = ( translation_.translation().isZero() && rotation_.matrix().isIdentity() );
  if (!is_msg_transform_identity)
  {
    ROS_FATAL("VoxbloxManager::registerPointcloudWithRoi: Input pointcloud should be in the sensor frame. Current frame: %s", msg->cloud.header.frame_id.c_str());
    ROS_FATAL("VoxbloxManager::registerPointcloudWithRoi: Skipping...");
    return false;
  }

  // Preprocess pointcloud and set PointXYZI where roi indices have high intensity values.
  // TODO: Doesn't work with PointXYZI. Worked with PointXYZRGB.
  pcl::PointCloud<pcl::PointXYZRGB> pc_xyzi;
  pcl::fromROSMsg(msg->cloud, pc_xyzi); // TODO: This line is printing "Failed to find match for field 'intensity'." if PointXYZI is used.
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
  tsdf_server->processPointCloudMessageAndInsert(pc2_xyzi, voxtransform, false);

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

  // TODO: check if point is in sampling region

  TargetVPtr new_roi_targets = std::make_shared<TargetV>();
  TargetVPtr new_expl_targets = std::make_shared<TargetV>();
  TargetVPtr new_border_targets = std::make_shared<TargetV>();

  // Iterate all allocated blocks
  auto layer = tsdf_server->tsdf_map_->getTsdfLayerConstPtr();
  size_t vps = layer->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;
  voxblox::BlockIndexList block_list;
  layer->getAllAllocatedBlocks(&block_list);
  for (const voxblox::BlockIndex& block_index : block_list)
  {
    // Iterate voxels
    const voxblox::Block<voxblox::TsdfVoxel>& block = layer->getBlockByIndex(block_index);
    for (size_t linear_index = 0u; linear_index < num_voxels_per_block; ++linear_index)
    {
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      const voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);

      octomap::point3d coord_octomap(coord.x(), coord.y(), coord.z());
      if (!isInSamplingRegion(coord_octomap, sr_min, sr_max))
        continue;

      voxblox::GlobalIndex g_idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_index, block.computeVoxelIndexFromLinearIndex(linear_index), vps);

      // Roi Target
      if (!can_skip_roi && voxblox::utils::isObservedVoxel(voxel) && voxel.color.r > 50 && voxel.distance>0) // TODO: constant. Note: voxel.distance>0 means the outer shell of the object surface
      {
        MappingKeySet neighbors;
        computeNeighborKeys(MappingKey(g_idx.x(), g_idx.y(), g_idx.z()), neighbors, neighbor_con);
        for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
        {
          const voxblox::TsdfVoxel* neighbor_voxel = layer->getVoxelPtrByGlobalIndex(toVoxbloxKey(*it));
          if (!neighbor_voxel || !voxblox::utils::isObservedVoxel(*neighbor_voxel)) // has unknown neighbor?
          {
            new_roi_targets->push_back(octomap::point3d(coord.x(), coord.y(), coord.z()));
            break;
          }
        }
      }

      // Border & Expl
      if (!(can_skip_border && can_skip_expl) && voxblox::utils::isObservedVoxel(voxel) && voxel.distance>0)
      {
        bool has_unknown_neighbor = false;
        bool has_occupied_neighbor = false;
        MappingKeySet neighbors;
        computeNeighborKeys(MappingKey(g_idx.x(), g_idx.y(), g_idx.z()), neighbors, neighbor_con);
        for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
        { // -----------for start
          const voxblox::TsdfVoxel* neighbor_voxel = layer->getVoxelPtrByGlobalIndex(toVoxbloxKey(*it));
          if (!neighbor_voxel)
          {
            has_unknown_neighbor = true;
            continue;
          }

          if (voxblox::utils::isObservedVoxel(*neighbor_voxel))
          {
            if (neighbor_voxel->distance < 0)
            {
              has_occupied_neighbor = true;
            }
          }
          else
          {
            has_unknown_neighbor = true;
          }
        } // -----------for end

        if (has_unknown_neighbor)
        {
          if (has_occupied_neighbor)
          {
            new_expl_targets->push_back(octomap::point3d(coord.x(), coord.y(), coord.z()));
          }
          else
          {
            new_border_targets->push_back(octomap::point3d(coord.x(), coord.y(), coord.z()));
          }
        }

      }

      // voxblox::BlockIndex neighbor_b_idx;
      // voxblox::BlockIndex neighbor_v_idx;
      // voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(toVoxbloxKey(*it), vps, &neighbor_b_idx, &neighbor_v_idx);
      // const voxblox::Block<voxblox::TsdfVoxel>& neighbor_block = layer->getBlockByIndex(neighbor_b_idx);
      // const voxblox::Point neighbor_coord = neighbor_block.computeCoordinatesFromVoxelIndex(neighbor_v_idx);
      // const voxblox::TsdfVoxel neighbor_voxel = neighbor_block.getVoxelByVoxelIndex(neighbor_v_idx);
      // new_border_targets->push_back(octomap::point3d(neighbor_coord.x(), neighbor_coord.y(), neighbor_coord.z()));

    }
  }

  roi_targets = new_roi_targets;
  border_targets = new_border_targets;
  expl_targets = new_expl_targets;
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
  //tsdf_server->updateMesh(); // publishes pointclouds, tsdf, and mesh
  //tsdf_server->generateMesh(); // publishes pointclouds, tsdf, and mesh
  //tsdf_server->publishPointclouds();
  tsdf_server->publishTsdfSurfacePoints();
  tsdf_mutex.unlock();
}

} // namespace view_motion_planner