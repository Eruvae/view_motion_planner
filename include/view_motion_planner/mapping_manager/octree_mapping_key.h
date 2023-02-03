#ifndef __OCTREE_MAPPING_KEY__
#define __OCTREE_MAPPING_KEY__

namespace view_motion_planner
{

// Inherits octomap::OcTreeKey, also provides BaseMappingKey interface

struct OctreeMappingKey : public BaseMappingKey, public octomap::OcTreeKey {
public:
  octomap::point3d getCoord() const
  {
    return octomap::point3d(k[0], k[1], k[2]);
  }

  bool operator== (const BaseMappingKey& other_key)
  {
    return dynamic_cast<const octomap::OcTreeKey&>(other_key) == dynamic_cast<const octomap::OcTreeKey&>(*this);
  }
};


} // namespace view_motion_planner

#endif // __OCTREE_MAPPING_KEY__