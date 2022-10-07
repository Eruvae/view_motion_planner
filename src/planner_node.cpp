#include <ros/ros.h>
#include <ros/package.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/LoadOctomap.h>
#include "view_motion_planner/view_motion_planner.h"



using namespace view_motion_planner;

ViewMotionPlanner *planner;

bool saveOctomap(roi_viewpoint_planner_msgs::SaveOctomap::Request &req, roi_viewpoint_planner_msgs::SaveOctomap::Response &res)
{
  if (req.specify_filename)
    res.filename = planner->getOctreeManager()->saveOctomap(req.name, req.name_is_prefix);
  else
    res.filename = planner->getOctreeManager()->saveOctomap();

  res.success = res.filename != "";
  return true;
}

bool loadOctomap(roi_viewpoint_planner_msgs::LoadOctomap::Request &req, roi_viewpoint_planner_msgs::LoadOctomap::Response &res)
{
  int err_code = planner->getOctreeManager()->loadOctomap(req.filename);
  res.success = (err_code == 0);
  if (err_code == -1) res.error_message = "Deserialization failed";
  else if (err_code == -2) res.error_message = "Wrong Octree type";
  return true;
}

bool resetOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  planner->getOctreeManager()->resetOctomap();
  return true;
}

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

  int eval_epend_param_int = nhp.param<int>("epend", 0);
  EvalEpisodeEndParam ep = EvalEpisodeEndParam::TIME;
  if (eval_epend_param_int == 1) ep = EvalEpisodeEndParam::PLAN_DURATION;
  else if (eval_epend_param_int == 2) ep = EvalEpisodeEndParam::PLAN_LENGTH;

  tf2_ros::Buffer tfBuffer(ros::Duration(30));
  tf2_ros::TransformListener tfListener(tfBuffer);

  planner = new ViewMotionPlanner(nh, tfBuffer, map_frame, ws_frame,
                                  robot_description_param_name, group_name, ee_link_name,
                                  tree_resolution, num_graph_builder_threads,
                                  update_planning_tree, evaluate_results, eval_num_episodes, ep, eval_episode_duration);

  ros::ServiceServer saveOctomapService = nh.advertiseService("/roi_viewpoint_planner/save_octomap", saveOctomap);
  ros::ServiceServer loadOctomapService = nh.advertiseService("/roi_viewpoint_planner/load_octomap", loadOctomap);
  ros::ServiceServer resetOctomapService = nh.advertiseService("/roi_viewpoint_planner/reset_octomap", resetOctomap);

  ROS_INFO_STREAM("PLANNER CREATED");
  planner->getRobotManager()->moveToHomePose();
  planner->plannerLoop();
}
