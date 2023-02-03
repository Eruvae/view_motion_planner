#include <view_motion_planner/visualization_utils.h>
#include <octomap_ros/conversions.h>
#include <string>

namespace view_motion_planner
{

visualization_msgs::Marker targetsToROSVisualizationMsg(const std::vector<octomap::point3d> &roi_targets, const std::vector<octomap::point3d> &expl_targets, const std::vector<octomap::point3d> &border_targets, std::string map_frame)
{
  const size_t NUM_P = roi_targets.size() + expl_targets.size() + border_targets.size();

  visualization_msgs::Marker m;
  m.header.frame_id = map_frame;
  m.header.stamp = ros::Time();
  m.ns = "targets";
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.005;
  m.scale.y = 0.005;
  m.color.a = 1.0;

  if (NUM_P == 0)
  {
    ROS_WARN("No targets. Returning an empty visualization msg!");
    return m;
  }

  m.points.reserve(NUM_P);
  m.colors.reserve(NUM_P);

  for (const octomap::point3d &p : roi_targets)
  {
    m.points.push_back(octomap::pointOctomapToMsg(p));
    m.colors.push_back(COLOR_RED);
  }
  for (const octomap::point3d &p : expl_targets)
  {
    m.points.push_back(octomap::pointOctomapToMsg(p));
    m.colors.push_back(COLOR_GREEN);
  }
  for (const octomap::point3d &p : border_targets)
  {
    m.points.push_back(octomap::pointOctomapToMsg(p));
    m.colors.push_back(COLOR_BLUE);
  }

  return m;
}

} // namespace view_motion_planner