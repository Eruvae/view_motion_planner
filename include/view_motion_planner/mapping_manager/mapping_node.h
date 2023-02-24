#ifndef __BASE_MAPPING_NODE__
#define __BASE_MAPPING_NODE__

namespace view_motion_planner
{

// Key state can be found using a mapping manager
enum MappingNodeState
{
  ERROR = -1, // state is not set. maybe the node is not found.
  UNKNOWN = 0, // unexplored
  FREE = 1, // not occupied
  NON_ROI = 2, // occupied by a non-roi cell
  ROI = 3 // occupied by a roi cell
};

struct MappingNode
{
  MappingNodeState state;
  float occ_p; // occupancy probability. valid if state is NON_ROI or ROI
  float roi_p; // roi probability. valid if state is NON_ROI or ROI

  MappingNode():state(ERROR), occ_p(std::nan("")), roi_p(std::nan("")){}
  bool isValid(){return (int)state > ERROR;}
};

} // namespace view_motion_planner

#endif // __BASE_MAPPING_NODE__