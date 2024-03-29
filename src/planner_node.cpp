#include <ros/ros.h>
#include <ros/package.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/LoadOctomap.h>
#include "view_motion_planner/view_motion_planner.h"
#include <roi_viewpoint_planner_msgs/SaveCurrentRobotState.h>
#include <roi_viewpoint_planner_msgs/StartEvaluator.h>
#include <sensor_msgs/JointState.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>

#include <urdf/model.h>
#include <srdfdom/srdf_writer.h>
#include "urdf_parser/urdf_parser.h"

namespace fs = std::filesystem;


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
  int err_code = planner->getOctreeManager()->loadOctomap(req.filename, req.offset);
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

bool saveCurrentRobotState(roi_viewpoint_planner_msgs::SaveCurrentRobotState::Request& req,roi_viewpoint_planner_msgs::SaveCurrentRobotState::Response& res)
{
  std::string package_path = ros::package::getPath	("phenorob_ur5e");
  ROS_INFO_STREAM("Package path: "<<package_path); 
  std::string srdf_file_location = package_path + "/config/platform_base/";
  std::string srdf_filename = "phenorob_ur5e.srdf";
  fs::copy(srdf_file_location + srdf_filename, srdf_file_location + "phenorob_ur5e_bkup.srdf", fs::copy_options::update_existing); // copy file

  ros::NodeHandle nh;
  std::string urdf_xml, srdf_xml, group_name;
  nh.getParam("/robot_description", urdf_xml); 
  nh.getParam("/robot_description_semantic", srdf_xml); 
  nh.param<std::string>("/roi_viewpoint_planner/group_name", group_name, "manipulator"); 
  ROS_INFO_STREAM("3");

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml);
  ROS_INFO_STREAM("4");
  srdf::SRDFWriter srdf_writer_;
  srdf::Model srdf_model; 
  srdf_model.initFile(*urdf_model, srdf_file_location + srdf_filename);
  ROS_INFO_STREAM("4.1");

  srdf_writer_.initModel(*urdf_model, srdf_model);

  boost::shared_ptr<sensor_msgs::JointState const> joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(std::string("/joint_states"), ros::Duration(1.0));
  srdf::Model::GroupState group_state;
  group_state.group_ = group_name;
  group_state.name_  = req.pose_name;

  const sensor_msgs::JointState js = *joint_state;

  for(unsigned i = 0; i < js.name.size(); ++i)
  {
    if(js.name[i].find("arm") != std::string::npos)
    {
        std::vector<double> value(1);
        value[0] = js.position[i];
        group_state.joint_values_.insert(std::pair<std::string, std::vector<double>>(js.name[i], value));
    }
  }
  srdf_writer_.group_states_.push_back(group_state);
  srdf_writer_.writeSRDF(srdf_file_location + srdf_filename);

  /*auto new_srdf_xml_string = srdf_writer_->getSRDFString();
  std::ofstream file(srdf_file_location + srdf_filename + "new");
  ROS_INFO_STREAM("1");
  file<<new_srdf_xml_string;
  ROS_INFO_STREAM("2");
  file.close();*/
  
  return true;
}

bool flipWsAndSr(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  planner->flipWsAndSr();
  return true;
}

bool updateWsAndSrMarker(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  planner->publishWorkspaceMarker();
  planner->publishSamplingRegionMarker();
  return true;
}

bool startEvaluator(roi_viewpoint_planner_msgs::StartEvaluator::Request &req, roi_viewpoint_planner_msgs::StartEvaluator::Response &res)
{
  if (req.episode_end_param >= static_cast<uint8_t>(EvalEpisodeEndParam::NUM_EPEND_PARAMS))
  {
    res.success = false;
  }
  else
  {
    EvalEpisodeEndParam epEndParam = static_cast<EvalEpisodeEndParam>(req.episode_end_param);
    res.success = planner->startEvaluator(req.num_evals, epEndParam, req.episode_duration, req.starting_index,
                  req.randomize_plants, octomap_vpp::pointToOctomath(req.min_point), octomap_vpp::pointToOctomath(req.max_point), req.min_dist, req.with_trolley);
  }
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
  std::string pose_frame = nh.param<std::string>("/roi_viewpoint_planner/pose_frame", "world");
  std::string robot_description_param_name = nh.param<std::string>("/roi_viewpoint_planner/robot_description_param_name", "robot_description");
  std::string group_name = nh.param<std::string>("/roi_viewpoint_planner/group_name", "manipulator");
  std::string ee_link_name = nh.param<std::string>("/roi_viewpoint_planner/ee_link_name", "camera_link");

  bool update_planning_tree = nhp.param<bool>("update_planning_tree", true);
  size_t num_graph_builder_threads = static_cast<size_t>(nhp.param<int>("graph_builder_threads", 4));
  bool initialize_evaluator = nhp.param<bool>("evaluate", true);

  tf2_ros::Buffer tfBuffer(ros::Duration(30));
  tf2_ros::TransformListener tfListener(tfBuffer);

  planner = new ViewMotionPlanner(nh, tfBuffer, map_frame, ws_frame, pose_frame,
                                  robot_description_param_name, group_name, ee_link_name,
                                  tree_resolution, num_graph_builder_threads,
                                  update_planning_tree, initialize_evaluator);

  ros::ServiceServer saveOctomapService = nh.advertiseService("/roi_viewpoint_planner/save_octomap", saveOctomap);
  ros::ServiceServer loadOctomapService = nh.advertiseService("/roi_viewpoint_planner/load_octomap", loadOctomap);
  ros::ServiceServer resetOctomapService = nh.advertiseService("/roi_viewpoint_planner/reset_octomap", resetOctomap);
  ros::ServiceServer saveCurrentRobotStateService = nh.advertiseService("/roi_viewpoint_planner/save_robot_state", saveCurrentRobotState);
  ros::ServiceServer flipWsAndSrService = nh.advertiseService("/roi_viewpoint_planner/flip_ws_and_sr", flipWsAndSr);
  ros::ServiceServer updateWsAndSrMarkerService = nh.advertiseService("/roi_viewpoint_planner/update_ws_and_sr_marker", updateWsAndSrMarker);
  ros::ServiceServer startEvaluatorService = nh.advertiseService("/roi_viewpoint_planner/start_evaluator", startEvaluator);


  ROS_INFO_STREAM("PLANNER CREATED");
  planner->getRobotManager()->moveToHomePose();
  planner->plannerLoop();
}
