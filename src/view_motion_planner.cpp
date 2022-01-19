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
    graph_manager(robot_manager),
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

void ViewMotionPlanner::graphVisualizeThread()
{
  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    graph_manager.getGraphMutex().lock_shared();
    ROS_INFO_STREAM("Computing connected components");
    ros::Time startTime = ros::Time::now();
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
    graph_manager.getGraphMutex().unlock_shared();
    visual_tools_->trigger();
    ROS_INFO_STREAM("Viewpose graph published, took " << (ros::Time::now() - startTime));
  }
}

void ViewMotionPlanner::generateViewposeGraph()
{
  observationPoseMtx.lock_shared();
  std::vector<Viewpose> vps = observationPoses;
  observationPoseMtx.unlock_shared();

  graph_manager.getGraphMutex().lock();
  graph_manager.clear();

  ROS_INFO_STREAM("Generating viewpose graph");
  ros::Time startTime(ros::Time::now());
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
  graph_manager.getGraphMutex().unlock();

  ROS_INFO_STREAM("Generating graph took " << (ros::Time::now() - startTime) << ", " << vps.size() << " poses inserted");
}

void ViewMotionPlanner::graphBuilderThread()
{
  std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
  while(true)
  {
    double target_selector = target_select_dist(random_engine);
    bool target_roi = target_selector < 0.8; // TODO: Make configurable
    Viewpose vp;
    bool success = octree_manager->sampleRandomViewPose(vp, target_roi, 0.3, 0.5); // TODO: Make configurable
    if (success)
    {
      graph_manager.getGraphMutex().lock();
      Vertex v = graph_manager.addViewpose(vp);
      graph_manager.connectNeighbors(v, 5, 2.0); // TODO: Make configurable
      graph_manager.getGraphMutex().unlock();
    }
  }
}

void ViewMotionPlanner::pathSearcherThread()
{

}

void ViewMotionPlanner::pathExecuterThead()
{

}

void ViewMotionPlanner::plannerLoop()
{
  boost::thread poseVisualizeThread(boost::bind(&ViewMotionPlanner::poseVisualizeThread, this));
  boost::thread graphVisualizeThread(boost::bind(&ViewMotionPlanner::graphVisualizeThread, this));
  std::vector<boost::thread> graphBuilderThreads;
  for (size_t i = 0; i < 8; i++)
  {
    graphBuilderThreads.push_back(boost::move(boost::thread(boost::bind(&ViewMotionPlanner::graphBuilderThread, this))));
  }
  ros::waitForShutdown();
  /*for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }*/
}

bool ViewMotionPlanner::plannerLoopOnce()
{
  std::vector<Viewpose> newPoses = octree_manager->sampleObservationPoses();
  observationPoseMtx.lock();
  observationPoses = newPoses;
  observationPoseMtx.unlock();
  octree_manager->publishObservationPoints(newPoses);
  generateViewposeGraph();
  return false;
}

} // namespace view_motion_planner
