#ifndef __VISUALIZATION_UTILS__
#define __VISUALIZATION_UTILS__

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap_types.h>

namespace view_motion_planner
{

// lambda initialization trick
static const std_msgs::ColorRGBA COLOR_RED = []{std_msgs::ColorRGBA c; c.r = 1.f; c.g = 0.f; c.b = 0.f; c.a = 1.f; return c; } ();
static const std_msgs::ColorRGBA COLOR_GREEN = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 1.f; c.b = 0.f; c.a = 1.f; return c; } ();
static const std_msgs::ColorRGBA COLOR_BLUE = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 0.f; c.b = 1.f; c.a = 1.f; return c; } ();


visualization_msgs::Marker targetsToROSVisualizationMsg(const std::vector<octomap::point3d> &roi_targets, 
                                                        const std::vector<octomap::point3d> &expl_targets, 
                                                        const std::vector<octomap::point3d> &border_targets,
                                                        std::string map_frame);



} // namespace view_motion_planner

#endif // __VISUALIZATION_UTILS__
