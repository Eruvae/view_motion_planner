//////////////////////////////////////
#include <view_motion_planner/mapping_manager/base_mapping_manager.h>
//////////////////////////////////////
#include <view_motion_planner/vmp_utils.h>

namespace view_motion_planner
{

void BaseMappingManager::computePoseObservedCells(const octomap::pose6d &pose, MappingKeySet &freeCells, MappingKeySet &occCells, MappingKeySet &unkCells)
{
  octomap::point3d_collection endpoints = computeVpRaycastEndpoints(pose);
  for (const octomap::point3d &end : endpoints)
  {
    MappingKeyRay ray;
    computeRayKeys(pose.trans(), end, ray);

    for (const MappingKey& key: ray)
    {
      MappingNode node = search(key);
      if (!node.isValid()) continue;

      if (node.state == MappingNodeState::UNKNOWN)
      {
        unkCells.insert(key);
      }
      else if (node.state == MappingNodeState::FREE)
      {
        freeCells.insert(key);
      }
      else if (node.state == MappingNodeState::NON_ROI || node.state == MappingNodeState::ROI)
      {
        occCells.insert(key);
      }
    }
  }
}

bool BaseMappingManager::computeRayNodes(const octomap::point3d& origin, const octomap::point3d& end, std::vector<MappingNode> &nodes)
  {
    MappingKeyRay ray;
    bool res = computeRayKeys(origin, end, ray);
    if (!res) return false;

    res = false;
    for (const MappingKey& key: ray)
    {
      MappingNode node = search(key);
      if (node.state > MappingNodeState::ERROR)
      {
        nodes.push_back(node);
        res = true;
      }
    }
    return res;
  }

} // namespace view_motion_planner