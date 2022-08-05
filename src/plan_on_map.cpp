#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "view_motion_planner/view_motion_planner.h"

using namespace view_motion_planner;

void updateTrolleyPosition(tf2_ros::TransformBroadcaster &trolley_br, ros::Publisher &js_pub, double pos, double height)
{
  ros::Time current_time = ros::Time::now();
  geometry_msgs::TransformStamped tp;
  tp.header.stamp = current_time;
  tp.header.frame_id = "map";
  tp.child_frame_id = "base_link";
  tp.transform.translation.x = pos;
  tp.transform.rotation.w = 1;

  sensor_msgs::JointState js;
  js.name.push_back("base_link_to_platform");
  js.position.push_back(height);
  js.header.stamp = current_time;

  trolley_br.sendTransform(tp);
  js_pub.publish(js);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "view_motion_planner");

  if (argc != 2)
  {
    ROS_ERROR("Please specify map file to load");
    return -1;
  }

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

  constexpr bool NO_PLANNING_TREE_UPDATE = false;

  size_t num_graph_builder_threads = static_cast<size_t>(nhp.param<int>("graph_builder_threads", 4));
  bool evaluate_results = nhp.param<bool>("evaluate", false);
  size_t eval_num_episodes = static_cast<size_t>(nhp.param<int>("episodes", 20));
  double eval_episode_duration = nhp.param<double>("duration", 120.0);

  tf2_ros::Buffer tfBuffer(ros::Duration(30));
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster trolley_br;
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("trolley_joint_states", 1);

  ViewMotionPlanner planner(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame,
                            robot_description_param_name, group_name, ee_link_name,
                            tree_resolution, num_graph_builder_threads,
                            NO_PLANNING_TREE_UPDATE, evaluate_results, eval_num_episodes, eval_episode_duration);
  ROS_INFO_STREAM("PLANNER CREATED");

  planner.getOctreeManager()->loadOctomap(argv[1]);

  planner.getConfig().mode = Vmp_PLAN;
  planner.updateConfig();

  size_t i = 0, j = 0;
  for(ros::Rate r(1); ros::ok(); r.sleep())
  {
    updateTrolleyPosition(trolley_br, js_pub, i*1.0, j*0.2);
    planner.pathSearcherThread(ros::Time::now() + ros::Duration(60));
    ++i %= 30;
    ++j %= 11;
  }

  ros::waitForShutdown();
}
