#include <view_motion_planner/mapping_manager/octree_manager.h>

namespace view_motion_planner
{

OctreeManager::OctreeManager(ros::NodeHandle nh, std::string frame_id): nh(nh), frame_id(frame_id), resolution(resolution)
{
  planningTree.reset(new octomap_vpp::RoiOcTree(resolution));
}

BaseMappingKeyRayPtr OctreeManager::computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end)
{
  BaseMappingKeyRayPtr ray = createEmptyKeyRay();

  tree_mtx.lock();
  octomap::KeyRay octomap_ray;
  bool ret = planningTree->computeRayKeys(origin, end, octomap_ray);
  tree_mtx.unlock();

  if (!ret)
  {
    return ray;
  }

  for (auto r : octomap_ray)
  {
    OctreeMappingKey key;
    key[0] = r[0];
    key[1] = r[1];
    key[2] = r[2];
    ray->push_back(key);
  }

  return ray;
}

bool OctreeManager::computeRayCells(const octomap::point3d& origin, const octomap::point3d& end, BaseMappingKeySetPtr &freeCells, BaseMappingKeySetPtr &occCells, BaseMappingKeySetPtr &unkCells)
{
  tree_mtx.lock();
  BaseMappingKeyRayPtr ray;
  BaseMappingKeyRayPtr ray = computeRayKeys(origin, end);

  if (ray->size() == 0)
  {
    tree_mtx.unlock();
    return false;
  }
  for (const BaseMappingKey &key : ray)
  {
    const octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
    octomap_vpp::NodeState state = planningTree->getNodeState(node, octomap_vpp::NodeProperty::OCCUPANCY);
    if (state == octomap_vpp::NodeState::FREE_NONROI)
      freeCells->insert(key);
    else if (state == octomap_vpp::NodeState::UNKNOWN)
      unkCells->insert(key);
    else
    {
      occCells->insert(key);
      break;
    }
  }
  tree_mtx.unlock();
  return true;
}


} // namespace view_motion_planner