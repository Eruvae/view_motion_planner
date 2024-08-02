#include "view_motion_planner/mapping_manager/panoptic_manager.h"
#include "view_motion_planner/vmp_utils.h"

namespace view_motion_planner
{

PanopticManager::PanopticManager(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, const std::string& map_frame, const std::string& ws_frame, tf2_ros::Buffer &tfBuffer, double resolution)
  : BaseMappingManager(nh, priv_nh, map_frame, ws_frame, tfBuffer), resolution(resolution), inv_resolution(1.0/resolution)
{
  map_mutex.lock();
  panoptic_mapper.reset(new panoptic_mapping::PanopticMapper(nh, priv_nh));
  map_mutex.unlock();
}

void 
PanopticManager::resetMap()
{
  ROS_WARN("Map reset not implemented"); 
  //map_mutex.lock();
  // TODO: getSubmapCollection returns const, maybe modify panoptic_mapping_ros later
  // panoptic_mapper->getSubmapCollection().clear();
  //map_mutex.unlock();
}

// TODO: not tested
MappingKey 
PanopticManager::coordToKey(const octomap::point3d &coord)
{
  voxblox::GlobalIndex vkey(coord.x()*inv_resolution, coord.y()*inv_resolution, coord.z()*inv_resolution);
  return toMappingKey(vkey);
}

// TODO: not tested
bool 
PanopticManager::computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, MappingKeyRay& ray)
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
PanopticManager::computeNeighborKeys(const MappingKey& point, MappingKeySet& set, const NeighborConnectivity connectivity)
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
PanopticManager::search(const MappingKey& key)
{
  // TODO: which ID to use? Loop through all IDs to identify class?
  size_t id = panoptic_mapper->getSubmapCollection().getActiveFreeSpaceSubmapID();
  const panoptic_mapping::Submap &submap = panoptic_mapper->getSubmapCollection().getSubmap(id);
  const voxblox::TsdfVoxel* voxel = submap.getTsdfLayer().getVoxelPtrByGlobalIndex(toVoxbloxKey(key));
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


/*bool 
PanopticManager::registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg, const geometry_msgs::Transform& pc_transform)
{
  ROS_WARN("Not implemented");
  return true;

  //TODO: this might not make sense to have, as panoptic mapper is supposed to have multiple classes

  //map_mutex.lock();
  //panoptic_mapping::InputData input_data;
  //panoptic_mapper->processInput(input_data);
  //map_mutex.unlock();
}*/


void 
PanopticManager::updateTargets(octomap::point3d sr_min, octomap::point3d sr_max, bool can_skip_roi, bool can_skip_expl, bool can_skip_border, NeighborConnectivity neighbor_con)
{
  std::scoped_lock lock(map_mutex, targets_mtx); // more robust to deadlocks

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

  // TODO: which ID to use? Loop through all IDs to identify class?
  const panoptic_mapping::SubmapCollection &submaps = panoptic_mapper->getThreadSafeSubmapCollection().getSubmaps();
  for (const panoptic_mapping::Submap &submap : submaps)
  {
    //LabelEntry
    //submap.getClassID();
    panoptic_mapping::LabelEntry entry;
    bool success = panoptic_mapper->getLabelHandler().getLabelEntryIfExists(submap.getClassID(), &entry);
    if (!success)
    {
      ROS_WARN_STREAM("No entry for class ID " << submap.getClassID());
      continue;
    }
    ROS_INFO_STREAM("Entry: " << entry.toString());
    bool is_roi_submap = (entry.name == "pepper");

    const auto &layer = submap.getTsdfLayer();
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;
    voxblox::BlockIndexList block_list;
    layer.getAllAllocatedBlocks(&block_list);
    for (const voxblox::BlockIndex& block_index : block_list)
    {
      // Iterate voxels
      const voxblox::Block<voxblox::TsdfVoxel>& block = layer.getBlockByIndex(block_index);
      for (size_t linear_index = 0u; linear_index < num_voxels_per_block; ++linear_index)
      {
        const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        const voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);

        octomap::point3d coord_octomap(coord.x(), coord.y(), coord.z());
        if (!isInSamplingRegion(coord_octomap, sr_min, sr_max))
          continue;

        voxblox::GlobalIndex g_idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_index, block.computeVoxelIndexFromLinearIndex(linear_index), vps);

        // Roi Target
        if (is_roi_submap && !can_skip_roi && voxblox::utils::isObservedVoxel(voxel) && voxel.distance>0) // TODO: constant. Note: voxel.distance>0 means the outer shell of the object surface
        {
          MappingKeySet neighbors;
          computeNeighborKeys(MappingKey(g_idx.x(), g_idx.y(), g_idx.z()), neighbors, neighbor_con);
          for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
          {
            const voxblox::TsdfVoxel* neighbor_voxel = layer.getVoxelPtrByGlobalIndex(toVoxbloxKey(*it));
            if (!neighbor_voxel || !voxblox::utils::isObservedVoxel(*neighbor_voxel)) // has unknown neighbor?
            {
              new_roi_targets->push_back(octomap::point3d(coord.x(), coord.y(), coord.z()));
              break;
            }
          }
        }

        // Border & Expl
        if (!is_roi_submap && !(can_skip_border && can_skip_expl) && voxblox::utils::isObservedVoxel(voxel) && voxel.distance>0)
        {
          bool has_unknown_neighbor = false;
          bool has_occupied_neighbor = false;
          MappingKeySet neighbors;
          computeNeighborKeys(MappingKey(g_idx.x(), g_idx.y(), g_idx.z()), neighbors, neighbor_con);
          for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
          { // -----------for start
            const voxblox::TsdfVoxel* neighbor_voxel = layer.getVoxelPtrByGlobalIndex(toVoxbloxKey(*it));
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

  }

  size_t id = panoptic_mapper->getThreadSafeSubmapCollection().getSubmaps().getActiveFreeSpaceSubmapID();
  const panoptic_mapping::Submap &submap = panoptic_mapper->getThreadSafeSubmapCollection().getSubmaps().getSubmap(id);
  

  roi_targets = new_roi_targets;
  border_targets = new_border_targets;
  expl_targets = new_expl_targets;
}


TargetVConstPtr 
PanopticManager::getRoiTargets()
{
  targets_mtx.lock();
  TargetVConstPtr tmp = roi_targets;
  targets_mtx.unlock();
  return tmp;
}


TargetVConstPtr 
PanopticManager::getExplTargets()
{
  targets_mtx.lock();
  TargetVConstPtr tmp = expl_targets;
  targets_mtx.unlock();
  return tmp;
}


TargetVConstPtr
PanopticManager::getBorderTargets()
{
  targets_mtx.lock();
  TargetVConstPtr tmp = border_targets;
  targets_mtx.unlock();
  return tmp;
}


bool 
PanopticManager::saveToFile(const std::string &filename, bool overwrite)
{
  return panoptic_mapper->saveMap(filename);
}

int 
PanopticManager::loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset)
{
  // TODO: offset
  map_mutex.lock();
  bool success = panoptic_mapper->loadMap(filename);
  map_mutex.unlock();

  if (!success) return -1;
  return 0;
}


void 
PanopticManager::publishMap()
{
  map_mutex.lock();
  panoptic_mapper->publishVisualization();
  map_mutex.unlock();
}

} // namespace view_motion_planner