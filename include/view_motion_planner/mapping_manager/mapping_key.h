#ifndef __BASE_MAPPING_KEY__
#define __BASE_MAPPING_KEY__

#include <unordered_set>
#include <vector>
#include <octomap/octomap_types.h>

using MappingKeyDtype = uint64_t;

namespace view_motion_planner
{

struct MappingKey
{
  MappingKeyDtype x, y, z;

  MappingKey(){}
  MappingKey(MappingKeyDtype x, MappingKeyDtype y, MappingKeyDtype z): x(x), y(y), z(z) {}

  bool operator== (const MappingKey& other_key) const
  {
    return (x==other_key.x) && (y==other_key.y) && (z==other_key.z);
  }

  // source: octomap/OcTreeKey.h
  struct KeyHash
  {
    size_t operator()(const MappingKey& key) const
    {
      // a simple hashing function 
      // explicit casts to size_t to operate on the complete range
      // constanst will be promoted according to C++ standard
      return static_cast<size_t>(key.x)
        + 1447*static_cast<size_t>(key.y)
        + 345637*static_cast<size_t>(key.z);
    }
  };
};

typedef std::shared_ptr<MappingKey> MappingKeyPtr;
typedef std::shared_ptr<MappingKey const> MappingKeyConstPtr;

typedef std::unordered_set<MappingKey, MappingKey::KeyHash> MappingKeySet; // Uses memory address for hashing, uses operator== for comparing items
typedef std::shared_ptr<MappingKeySet> MappingKeySetPtr;
typedef std::shared_ptr<MappingKeySet const> MappingKeySetConstPtr;

typedef std::vector<MappingKey> MappingKeyRay;
typedef std::shared_ptr<MappingKeyRay> MappingKeyRayPtr;
typedef std::shared_ptr<MappingKeyRay const> MappingKeyRayConstPtr;


} // namespace view_motion_planner

#endif // __BASE_MAPPING_KEY__