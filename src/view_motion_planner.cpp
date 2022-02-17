#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                  const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator)
  : random_engine(std::random_device{}()),
    config_server(config_mutex),
    vt_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_graph")),
    vt_searched_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_searched_graph")),
    robot_manager(new RobotManager(nh, tfBuffer, map_frame)),
    vt_robot_state(new moveit_visual_tools::MoveItVisualTools(map_frame, "vm_robot_state", robot_manager->getPlanningSceneMonitor())),
    octree_manager(new OctreeManager(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, random_engine, robot_manager, config, 100, initialize_evaluator)),
    graph_manager(new ViewposeGraphManager(robot_manager, octree_manager, vt_searched_graph, config))
{
  config_server.setCallback(boost::bind(&ViewMotionPlanner::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2));
  vt_robot_state->loadMarkerPub(true);
  vt_robot_state->loadRobotStatePub("planned_state");
  vt_robot_state->setManualSceneUpdating();
}

void ViewMotionPlanner::reconfigureCallback(VmpConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure callback called, level " << level);

  // Adjust sensor range if necessary
  if (level & (1 << 5)) // minimum sensor range
  {
    if (config.sensor_max_range < config.sensor_min_range)
      config.sensor_max_range = config.sensor_min_range;
  }
  else if (level & (1 << 6)) // maximum sensor range
  {
    if (config.sensor_min_range > config.sensor_max_range)
      config.sensor_min_range = config.sensor_max_range;
  }

  // Adjust target ratio if necessary
  double ratio_diff = 1 - (config.roi_target_ratio + config.expl_target_ratio + config.border_target_ratio);
  if (ratio_diff != 0.0)
  {
    if (level & (1 << 9)) // roi_target_ratio
    {
      double other_sum = config.expl_target_ratio + config.border_target_ratio;
      double expl_diff, border_diff;
      if (other_sum != 0.0)
      {
        expl_diff = ratio_diff * config.expl_target_ratio / other_sum;
        border_diff = ratio_diff * config.border_target_ratio / other_sum;
      }
      else
      {
        expl_diff = border_diff = 0.5 * ratio_diff;
      }
      config.expl_target_ratio += expl_diff;
      config.border_target_ratio += border_diff;
    }
    else if (level & (1 << 10)) // expl_target_ratio
    {
      config.border_target_ratio += ratio_diff;
      if (config.border_target_ratio < 0)
      {
        config.roi_target_ratio += config.border_target_ratio;
        config.border_target_ratio = 0;
      }
    }
    else if (level & (1 << 11)) // border_target_ratio
    {
      config.expl_target_ratio += ratio_diff;
      if (config.expl_target_ratio < 0)
      {
        config.roi_target_ratio += config.expl_target_ratio;
        config.expl_target_ratio = 0;
      }
    }
  }
  this->config = config;
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

/*void ViewMotionPlanner::generateViewposeGraph()
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
    graph_manager->connectNeighbors(*vi, 5, 2.0);
  }
  ViewposePtr cam_vp(new Viewpose());
  cam_vp->state = robot_manager->getCurrentState();
  cam_vp->pose = robot_manager->getCurrentPose();
  Vertex cam_vert = graph_manager->addViewpose(cam_vp);
  graph_manager->connectNeighbors(cam_vert, 5, 2.0);

  ROS_INFO_STREAM("Generating graph took " << (ros::Time::now() - startTime) << ", " << vps.size() << " poses inserted");
}*/

void ViewMotionPlanner::graphBuilderThread()
{
  //static const size_t MAX_GRAPH_VERTICES = 10000;
  std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
  while(ros::ok())
  {
    graphBuilderPaused.wait();

    //ROS_INFO_STREAM("Vertices in graph: " << boost::num_vertices(graph_manager->getGraph()));
    //if(boost::num_vertices(graph_manager->getGraph()) > MAX_GRAPH_VERTICES)
    //  break;

    double target_selector = target_select_dist(random_engine);
    TargetType type;
    if (target_selector <= config.roi_target_ratio)
      type = TARGET_ROI;
    else if (target_selector <= config.roi_target_ratio + config.expl_target_ratio)
      type = TARGET_OCC;
    else
      type = TARGET_BORDER;

    ViewposePtr vp = octree_manager->sampleRandomViewPose(type, config.sensor_min_range, config.sensor_max_range);
    if (vp)
    {
      boost::unique_lock lock(graph_manager->getGraphMutex());
      Vertex v = graph_manager->addViewpose(vp);
      graph_manager->connectNeighbors(v, static_cast<size_t>(config.max_nb_connect_count), config.max_nb_connect_dist);
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
  Vertex cam_vert = initCameraPoseGraph();
  graph_manager->initStartPose(cam_vert);
  ros::Duration(config.graph_building_time).sleep();

  for (ros::Rate rate(1); ros::ok(); rate.sleep()) // connect start pose if not connected
  {
    boost::unique_lock lock(graph_manager->getGraphMutex());
    graph_manager->connectNeighbors(cam_vert, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
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
      if (!success || (ros::Time::now() - start_expand).toSec() > config.graph_search_time)
        break;
    }
    graph_manager->visualizeGraph(config.visualize_expanded, config.visualize_unexpanded);

    std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr>> trajectories = graph_manager->getNextTrajectories(config.execution_cost_limit);
    Vertex next_start_vertex = graph_manager->getCurrentStartVertex();
    bool moved = false;
    ROS_INFO_STREAM("Executing " << trajectories.size() << " trajectories");
    for (const auto &[next_vertex, traj] : trajectories)
    {
      if (!traj)
      {
        ROS_WARN_STREAM("No trajectory found");
        graph_manager->connectNeighbors(next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
        break;
      }
      else
      {
        if (moved)
        {
          if (!robot_manager->isValidTrajectory(traj))
          {
            ROS_WARN_STREAM("Planned trajectory no longer valid");
            break;
          }
        }
        bool success = robot_manager->executeTrajectory(traj);
        moved = true;
        graph_manager->markAsVisited(next_start_vertex);
        octree_manager->waitForPointcloudWithRoi();
        if (!success)
        {
          ROS_WARN_STREAM("Failed to execute trajectory, setting new start pose");
          next_start_vertex = initCameraPoseGraph();
          graph_manager->connectNeighbors(cam_vert, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
          break;
        }
        else
        {
          next_start_vertex = next_vertex;
        }
      }
    }
    if (moved)
    {
      graph_manager->cleanupAfterMove();
      graph_manager->initStartPose(next_start_vertex);
    }
    resumeGraphBuilderThreads();
    ros::Duration(config.graph_building_time).sleep();
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

/*bool ViewMotionPlanner::plannerLoopOnce()
{
  std::vector<ViewposePtr> newPoses = octree_manager->sampleObservationPoses();
  observationPoseMtx.lock();
  observationPoses = newPoses;
  observationPoseMtx.unlock();
  octree_manager->publishObservationPoints(newPoses);
  //generateViewposeGraph();
  return false;
}*/

} // namespace view_motion_planner
