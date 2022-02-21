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
  bool update_planning_tree = nh.param<bool>("/roi_viewpoint_planner/update_planning_tree", true);
  bool initialize_evaluator = nh.param<bool>("/roi_viewpoint_planner/initialize_evaluator", true);

  size_t num_graph_builder_threads = static_cast<size_t>(nhp.param<int>("graph_builder_threads", 4));
  bool evaluate_results = nhp.param<bool>("evaluate_results", false);

  tf2_ros::Buffer tfBuffer(ros::Duration(30));
  tf2_ros::TransformListener tfListener(tfBuffer);

  ViewMotionPlanner planner(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, num_graph_builder_threads, evaluate_results);

  std::vector<double> joint_start_values;
  if(nh.getParam("/roi_viewpoint_planner/initial_joint_values", joint_start_values))
  {
    planner.getRobotManager()->moveToState(joint_start_values);
  }
  else
  {
    ROS_WARN("No inital joint values set");
  }

  planner.plannerLoop();
}
