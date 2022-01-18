#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                  const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator)
  : random_engine(std::random_device{}()),
    robot_manager(new RobotManager(nh, tfBuffer, map_frame)),
    octree_manager(new OctreeManager(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, random_engine, robot_manager, 100, initialize_evaluator)),
    visual_tools_(new moveit_visual_tools::MoveItVisualTools(map_frame, "visual_markers", robot_manager->getPlanningSceneMonitor()))
{
  visual_tools_->loadMarkerPub(true);
  visual_tools_->loadRobotStatePub("planned_state");
  visual_tools_->setManualSceneUpdating();
}

MoveGroupInterface::Plan ViewMotionPlanner::getNextPlan()
{
  MoveGroupInterface::Plan plan;
  //TODO: compute plan
  return plan;
}

void ViewMotionPlanner::poseVisualizeThread()
{
  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    observationPoseMtx.lock_shared();
    if (observationPoses.size() > 0)
    {
      std::uniform_int_distribution<size_t> distribution(0, observationPoses.size() - 1);
      size_t i = distribution(random_engine);
      const Viewpose &vp = observationPoses[i];
      visual_tools_->publishRobotState(vp.state);
    }
    observationPoseMtx.unlock_shared();
  }
}

void ViewMotionPlanner::generateViewposeGraph()
{
  observationPoseMtx.lock_shared();
  std::vector<Viewpose> vps = observationPoses;
  observationPoseMtx.unlock_shared();

  ROS_INFO_STREAM("Generating viewpose graph");
  ros::Time startTime(ros::Time::now());
  ViewposeGraphManager graph_manager(robot_manager);
  for (const Viewpose &vp : vps)
  {
    graph_manager.addViewpose(vp);
  }
  for (auto [vi, vi_end] = boost::vertices(graph_manager.getGraph()); vi != vi_end; vi++)
  {
    graph_manager.connectNeighbors(*vi, 5, DBL_MAX);
  }
  Viewpose cam_vp;
  cam_vp.state = robot_manager->getCurrentState();
  cam_vp.pose = robot_manager->getCurrentPose();
  Vertex cam_vert = graph_manager.addViewpose(cam_vp);
  graph_manager.connectNeighbors(cam_vert, 5, DBL_MAX);

  ROS_INFO_STREAM("Generating graph took " << (ros::Time::now() - startTime) << ", " << vps.size() << " poses inserted");
  ROS_INFO_STREAM("Computing connected components");
  startTime = ros::Time::now();
  using VPCMap = std::unordered_map<Vertex, size_t>;
  using VPCPropertyMap = boost::associative_property_map<VPCMap>;
  VPCMap component_map;
  VPCPropertyMap component_property_map(component_map);
  size_t num = boost::connected_components(graph_manager.getGraph(), component_property_map);
  ROS_INFO_STREAM("Computing connected components took " << (ros::Time::now() - startTime) << ", " << num << " components");
  startTime = ros::Time::now();
  visual_tools_->deleteAllMarkers();
  for (auto [ei, ei_end] = boost::edges(graph_manager.getGraph()); ei != ei_end; ei++)
  {
    visual_tools_->publishLine(graph_manager.getGraph()[ei->m_source].pose.position, graph_manager.getGraph()[ei->m_target].pose.position, visual_tools_->intToRvizColor(component_map[ei->m_source] % 15));
    // visual_tools_->publishTrajectoryLine(graph_manager.getGraph()[*ei].traj, robot_manager->getJointModelGroup(), visual_tools_->intToRvizColor(component_map[ei->m_source] % 15));
  }
  visual_tools_->trigger();
  ROS_INFO_STREAM("Viewpose graph published, took " << (ros::Time::now() - startTime));
}

void ViewMotionPlanner::plannerLoop()
{
  boost::thread visualizeThread(boost::bind(&ViewMotionPlanner::poseVisualizeThread, this));
  for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }
}

bool ViewMotionPlanner::plannerLoopOnce()
{
  std::vector<Viewpose> newPoses = octree_manager->sampleObservationPoses();
  observationPoseMtx.lock();
  observationPoses = newPoses;
  observationPoseMtx.unlock();

  octree_manager->publishMap();
  octree_manager->publishObservationPoints(newPoses);


  generateViewposeGraph();
  return false;
}

} // namespace view_motion_planner
