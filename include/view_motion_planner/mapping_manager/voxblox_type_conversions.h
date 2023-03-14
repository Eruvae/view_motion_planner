#ifndef __VOXBLOX_TYPE_CONVERSIONS__
#define __VOXBLOX_TYPE_CONVERSIONS__

#include "mapping_key.h"
#include <voxblox/core/common.h>

namespace view_motion_planner
{

///////////////////////////////////////////////////////////////////////////////////
// FROM VOXBLOX KEY TYPE TO GENERALIZED KEY TYPE
///////////////////////////////////////////////////////////////////////////////////

static inline MappingKey toMappingKey(const voxblox::GlobalIndex& key)
{
  return MappingKey(key(0), key(1), key(2));
}

// static inline MappingKeySet toMappingKeySet(const octomap::KeySet& set)
// {
//   MappingKeySet out_set;
//   for (auto& key: set)
//   {
//     out_set.insert(toMappingKey(key));
//   }
//   return out_set;
// }


// static inline MappingKeyRay toMappingKeyRay(const octomap::KeyRay& ray)
// {
//   MappingKeyRay out_ray;
//   for (auto& key: ray)
//   {
//     out_ray.push_back(toMappingKey(key));
//   }
//   return out_ray;
// }

///////////////////////////////////////////////////////////////////////////////////
// FROM GENERALIZED KEY TYPE TO VOXBLOX KEY TYPE
///////////////////////////////////////////////////////////////////////////////////

static inline voxblox::GlobalIndex toVoxbloxKey(const MappingKey& key)
{
  return voxblox::GlobalIndex(key.x, key.y, key.z);
}

// static inline octomap::KeySet toOcTreeKeySet(const MappingKeySet& set)
// {
//   octomap::KeySet out_set;
//   for (auto& key: set)
//   {
//     out_set.insert(toOcTreeKey(key));
//   }
//   return out_set;
// }

// static inline octomap::KeyRay toOcTreeKeyRay(const MappingKeyRay& ray)
// {
//   octomap::KeyRay out_ray;
//   for (auto& key: ray)
//   {
//     out_ray.addKey(toOcTreeKey(key)); // octomap::KeyRay is not wrapped with std::vector, instead has its own method
//   }
//   return out_ray;
// }

///////////////////////////////////////////////////////////////////////////////////

} // namespace view_motion_planner

#endif // __VOXBLOX_TYPE_CONVERSIONS__