#ifndef __VISUALIZATION_UTILS__
#define __VISUALIZATION_UTILS__

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap_types.h>

namespace view_motion_planner
{

visualization_msgs::Marker targetsToROSVisualizationMsg(const std::vector<octomap::point3d> &roi_targets, 
                                                        const std::vector<octomap::point3d> &expl_targets, 
                                                        const std::vector<octomap::point3d> &border_targets,
                                                        std::string map_frame);



} // namespace view_motion_planner

#endif // __VISUALIZATION_UTILS__
