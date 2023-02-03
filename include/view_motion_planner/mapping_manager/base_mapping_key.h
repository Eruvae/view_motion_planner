#ifndef __BASE_MAPPING_KEY__
#define __BASE_MAPPING_KEY__

#include <unordered_set>
#include <vector>
#include <octomap/octomap_types.h>

namespace view_motion_planner
{

struct BaseMappingKey {
public:
  /// @brief Position of the key w.r.t. the frame of the map where this key belongs to.
  /// @return 
  virtual octomap::point3d getCoord() const = 0;

  /// @brief Don't mix different key types!
  /// @param other_key Another key with the same type.
  /// @return True, if equals.
  virtual bool operator== (const BaseMappingKey& other_key) = 0;
};

typedef std::shared_ptr<BaseMappingKey> BaseMappingKeyPtr;

typedef std::unordered_set<BaseMappingKey> BaseMappingKeySet; // Uses memory address for hashing, uses operator== for comparing items
typedef std::shared_ptr<BaseMappingKeySet> BaseMappingKeySetPtr;

typedef std::vector<BaseMappingKey> BaseMappingKeyRay;
typedef std::shared_ptr<BaseMappingKeyRay> BaseMappingKeyRayPtr;

} // namespace view_motion_planner


#endif // __BASE_MAPPING_KEY__