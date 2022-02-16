#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                  const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator)
  : random_engine(std::random_device{}()),
    vt_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_graph")),
    vt_searched_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_searched_graph")),
    robot_manager(new RobotManager(nh, tfBuffer, map_frame)),
    vt_robot_state(new moveit_visual_tools::MoveItVisualTools(map_frame, "vm_robot_state", robot_manager->getPlanningSceneMonitor())),
    octree_manager(new OctreeManager(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, random_engine, robot_manager, 100, initialize_evaluator)),
    graph_manager(new ViewposeGraphManager(robot_manager, octree_manager, vt_searched_graph))
{
  vt_robot_state->loadMarkerPub(true);
  vt_robot_state->loadRobotStatePub("planned_state");
  vt_robot_state->setManualSceneUpdating();
}

MoveGroupInterface::Plan ViewMotionPlanner::getNextPlan()
{
  MoveGroupInterface::Plan plan;
  //TODO: compute plan
  return plan;
}

void ViewMotionPlanner::poseVisualizeThread()
{
  for (ros::Rate rate(0.2); ros::ok(); rate.sleep())
  {
    observationPoseMtx.lock_shared();
    if (observationPoses.size() > 0)
    {
      std::uniform_int_distribution<size_t> distribution(0, observationPoses.size() - 1);
      size_t i = distribution(random_engine);
      ViewposePtr vp = observationPoses[i];

      vt_robot_state->publishRobotState(vp->state);

      //octomap::pose6d viewpose = robot_manager->getRobotStatePose(vp.state);
      //geometry_msgs::Point vpm = octomap::pointOctomapToMsg(viewpose.trans());
      //octomap::point3d_collection endpoints = computeVpRaycastEndpoints(viewpose);

      octomap::pose6d viewpose = octomap_vpp::poseToOctomath(vp->pose);
      geometry_msgs::Point vpm = vp->pose.position;
      octomap::point3d_collection endpoints = computeVpRaycastEndpoints(viewpose);

      vt_robot_state->deleteAllMarkers();
      for (const octomap::point3d &ep : endpoints)
      {
        geometry_msgs::Point epm = octomap::pointOctomapToMsg(ep);
        vt_robot_state->publishLine(vpm, epm, rviz_visual_tools::RvizVisualTools::intToRvizColor(0));
      }
      vt_robot_state->trigger();
    }
    observationPoseMtx.unlock_shared();
  }
}

void ViewMotionPlanner::graphVisualizeThread()
{
  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    boost::shared_lock lock(graph_manager->getGraphMutex());
    ROS_INFO_STREAM("Computing connected components");
    ros::Time startTime = ros::Time::now();
    using VPCMap = std::unordered_map<Vertex, size_t>;
    using VPCPropertyMap = boost::associative_property_map<VPCMap>;
    VPCMap component_map;
    VPCPropertyMap component_property_map(component_map);
    size_t num = boost::connected_components(graph_manager->getGraph(), component_property_map);
    ROS_INFO_STREAM("Computing connected components took " << (ros::Time::now() - startTime) << ", " << num << " components");
    startTime = ros::Time::now();
    vt_graph->deleteAllMarkers();
    for (auto [ei, ei_end] = boost::edges(graph_manager->getGraph()); ei != ei_end; ei++)
    {
      vt_graph->publishLine(graph_manager->getGraph()[ei->m_source]->pose.position, graph_manager->getGraph()[ei->m_target]->pose.position, rviz_visual_tools::RvizVisualTools::intToRvizColor(component_map[ei->m_source] % 15));
      // visual_tools_->publishTrajectoryLine(graph_manager->getGraph()[*ei].traj, robot_manager->getJointModelGroup(), visual_tools_->intToRvizColor(component_map[ei->m_source] % 15));
    }
    vt_graph->trigger();
    ROS_INFO_STREAM("Viewpose graph published, took " << (ros::Time::now() - startTime));
  }
}

void ViewMotionPlanner::generateViewposeGraph()
{
  observationPoseMtx.lock_shared();
  std::vector<ViewposePtr> vps = observationPoses;
  observationPoseMtx.unlock_shared();

  boost::unique_lock lock(graph_manager->getGraphMutex());
  graph_manager->clear();

  ROS_INFO_STREAM("Generating viewpose graph");
  ros::Time startTime(ros::Time::now());
  for (const ViewposePtr &vp : vps)
  {
    graph_manager->addViewpose(vp);
  }
  for (auto [vi, vi_end] = boost::vertices(graph_manager->getGraph()); vi != vi_end; vi++)
  {
    graph_manager->connectNeighbors(*vi, 5, DBL_MAX);
  }
  ViewposePtr cam_vp(new Viewpose());
  cam_vp->state = robot_manager->getCurrentState();
  cam_vp->pose = robot_manager->getCurrentPose();
  Vertex cam_vert = graph_manager->addViewpose(cam_vp);
  graph_manager->connectNeighbors(cam_vert, 5, DBL_MAX);

  ROS_INFO_STREAM("Generating graph took " << (ros::Time::now() - startTime) << ", " << vps.size() << " poses inserted");
}

void ViewMotionPlanner::graphBuilderThread()
{
  static const size_t MAX_GRAPH_VERTICES = 10000;
  std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
  while(ros::ok())
  {
    graphBuilderPaused.wait();

    if(boost::num_vertices(graph_manager->getGraph()) > MAX_GRAPH_VERTICES)
      break;

    double target_selector = target_select_dist(random_engine);
    bool target_roi = target_selector < 0.8; // TODO: Make configurable
    ViewposePtr vp = octree_manager->sampleRandomViewPose(target_roi, 0.3, 0.5); // TODO: Make configurable
    if (vp)
    {
      boost::unique_lock lock(graph_manager->getGraphMutex());
      Vertex v = graph_manager->addViewpose(vp);
      graph_manager->connectNeighbors(v, 5, 2.0); // TODO: Make configurable
    }
  }
}

void ViewMotionPlanner::computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells)
{
  octomap::pose6d viewpose = robot_manager->getRobotStatePose(state);
  octree_manager->computePoseObservedCells(viewpose, freeCells, occCells, unkCells);
}

Vertex ViewMotionPlanner::initCameraPoseGraph()
{
  boost::unique_lock lock(graph_manager->getGraphMutex());
  ViewposePtr cam_vp(new Viewpose());
  cam_vp->state = robot_manager->getCurrentState();
  cam_vp->pose = robot_manager->getCurrentPose();
  Vertex cam_vert = graph_manager->addViewpose(cam_vp);
  return cam_vert;
}

void ViewMotionPlanner::pathSearcherThread()
{
  const double GRAPH_BUILDING_TIME = 2;
  const double GRAPH_SEARCH_TIME = 2;

  Vertex cam_vert = initCameraPoseGraph();
  graph_manager->initStartPose(cam_vert);
  ros::Duration(GRAPH_BUILDING_TIME).sleep();

  for (ros::Rate rate(1); ros::ok(); rate.sleep()) // connect start pose if not connected
  {
    boost::unique_lock lock(graph_manager->getGraphMutex());
    graph_manager->connectNeighbors(cam_vert, 5, DBL_MAX);
    size_t num_out_edges = boost::out_degree(cam_vert, graph_manager->getGraph());
    ROS_INFO_STREAM("Number of out edges from cam: " << num_out_edges);
    if (num_out_edges > 0)
      break;
  }

  while (ros::ok())
  {
    pauseGraphBuilderThreads();
    ros::Time start_expand(ros::Time::now());
    while (ros::ok())
    {
      bool success = graph_manager->expand();
      if (!success || (ros::Time::now() - start_expand).toSec() < GRAPH_SEARCH_TIME)
        break;
    }
    graph_manager->visualizeGraph();
    auto [next_vertex, traj] = graph_manager->getNextTrajectory();
    if (!traj)
    {
      ROS_ERROR_STREAM("No trajectory found");
    }
    else
    {
      robot_manager->executeTrajectory(traj);
      octree_manager->waitForPointcloudWithRoi();

      graph_manager->cleanupAfterMove(next_vertex);
      //cam_vert = initCameraPoseGraph();
      //graph_manager->connectNeighbors(cam_vert, 5, 2.0);
      //graph_manager->initStartPose(cam_vert);
    }
    resumeGraphBuilderThreads();
    ros::Duration(GRAPH_BUILDING_TIME).sleep();
  }
}

void ViewMotionPlanner::pathExecuterThead()
{

}

void ViewMotionPlanner::initGraphBuilderThreads()
{
  for (boost::thread &t : graphBuilderThreads)
  {
    t = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::graphBuilderThread, this)));
  }
}

void ViewMotionPlanner::pauseGraphBuilderThreads()
{
  graphBuilderPaused.setPaused(true);
}

void ViewMotionPlanner::resumeGraphBuilderThreads()
{
  graphBuilderPaused.setPaused(false);
}

void ViewMotionPlanner::plannerLoop()
{
  octree_manager->waitForPointcloudWithRoi();
  boost::thread poseVisualizeThread(boost::bind(&ViewMotionPlanner::poseVisualizeThread, this));
  //boost::thread graphVisualizeThread(boost::bind(&ViewMotionPlanner::graphVisualizeThread, this));

  initGraphBuilderThreads();
  boost::thread pathSearcherThread(boost::bind(&ViewMotionPlanner::pathSearcherThread, this));
  ros::waitForShutdown();
  /*for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }*/
}

bool ViewMotionPlanner::plannerLoopOnce()
{
  std::vector<ViewposePtr> newPoses = octree_manager->sampleObservationPoses();
  observationPoseMtx.lock();
  observationPoses = newPoses;
  observationPoseMtx.unlock();
  octree_manager->publishObservationPoints(newPoses);
  generateViewposeGraph();
  return false;
}

} // namespace view_motion_planner
