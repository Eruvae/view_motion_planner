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
  std::string wstree_file = nhp.param<std::string>("workspace_tree", wstree_default_package + "/workspace_trees/static/workspace_map.ot");
  std::string sampling_tree_file = nhp.param<std::string>("sampling_tree", wstree_default_package + "/workspace_trees/static/inflated_ws_tree.ot");
  std::string map_frame = nhp.param<std::string>("map_frame", "world");
  std::string ws_frame = nhp.param<std::string>("ws_frame", "arm_base_link");
  bool update_planning_tree = nhp.param<bool>("update_planning_tree", true);
  bool initialize_evaluator = nhp.param<bool>("initialize_evaluator", true);

  bool evaluate_results = nhp.param<bool>("evaluate_results", false);

  tf2_ros::Buffer tfBuffer(ros::Duration(30));
  tf2_ros::TransformListener tfListener(tfBuffer);

  ViewMotionPlanner planner(nh, tfBuffer, map_frame, tree_resolution, evaluate_results);
}
