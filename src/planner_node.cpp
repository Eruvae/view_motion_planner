#include <ros/ros.h>
#include <ros/package.h>
#include "view_motion_planner/view_motion_planner.h"

using namespace view_motion_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "view_motion_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  double tree_resolution = 0.01;
  if (nhp.hasParam("tree_resolution"))
    nhp.getParam("tree_resolution", tree_resolution);
  else
    nhp.setParam("tree_resolution", 0.01);

  std::string wstree_default_package = ros::package::getPath("ur_with_cam_gazebo");
  std::string wstree_file = nh.param<std::string>("/roi_viewpoint_planner/workspace_tree", wstree_default_package + "/workspace_trees/static/workspace_map.ot");
  std::string sampling_tree_file = nh.param<std::string>("/roi_viewpoint_planner/sampling_tree", wstree_default_package + "/workspace_trees/static/inflated_ws_tree.ot");
  std::string map_frame = nh.param<std::string>("/roi_viewpoint_planner/map_frame", "world");
  std::string ws_frame = nh.param<std::string>("/roi_viewpoint_planner/ws_frame", "arm_base_link");
  std::string robot_description_param_name = nh.param<std::string>("/roi_viewpoint_planner/robot_description_param_name", "robot_description");
  std::string group_name = nh.param<std::string>("/roi_viewpoint_planner/group_name", "manipulator");
  std::string ee_link_name = nh.param<std::string>("/roi_viewpoint_planner/ee_link_name", "camera_link");

  bool update_planning_tree = nhp.param<bool>("update_planning_tree", true);
  size_t num_graph_builder_threads = static_cast<size_t>(nhp.param<int>("graph_builder_threads", 4));
  bool evaluate_results = nhp.param<bool>("evaluate", false);
  size_t eval_num_episodes = static_cast<size_t>(nhp.param<int>("episodes", 20));
  double eval_episode_duration = nhp.param<double>("duration", 120.0);

  tf2_ros::Buffer tfBuffer(ros::Duration(30));
  tf2_ros::TransformListener tfListener(tfBuffer);

  ViewMotionPlanner planner(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame,
                            robot_description_param_name, group_name, ee_link_name,
                            tree_resolution, num_graph_builder_threads,
                            update_planning_tree, evaluate_results, eval_num_episodes, eval_episode_duration);
  ROS_INFO_STREAM("PLANNER CREATED");
  planner.getRobotManager()->moveToHomePose();
  planner.plannerLoop();
}
