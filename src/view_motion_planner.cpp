#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>
#include <octomap_ros/conversions.h>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer,
                                     const std::string &map_frame, const std::string &ws_frame, const std::string &robot_description_param_name,
                                     const std::string &group_name, const std::string &ee_link_name, double tree_resolution, size_t graph_builder_threads,
                                     bool update_planning_tree, bool evaluation_mode, size_t eval_num_episodes, EvalEpisodeEndParam ep, double eval_episode_duration)
  : nh_(nh),
    priv_nh_("~"),
    tfBuffer(tfBuffer),
    random_engine(std::random_device{}()),
    map_frame(map_frame),
    ws_frame(ws_frame),
    NUM_GRAPH_BUILDER_THREADS(graph_builder_threads),
    update_planning_tree(update_planning_tree),
    evaluation_mode(evaluation_mode),
    eval_num_episodes(eval_num_episodes),
    ep(ep),
    eval_episode_duration(eval_episode_duration),
    config_server(config_mutex),
    vt_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_graph")),
    vt_searched_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_searched_graph")),
    robot_manager(new RobotManager(nh, tfBuffer, map_frame, robot_description_param_name, group_name, ee_link_name)),
    vt_robot_state(new moveit_visual_tools::MoveItVisualTools(map_frame, "vm_robot_state", robot_manager->getPlanningSceneMonitor())),
    trolley_remote(ros::NodeHandle(), ros::NodeHandle("/trollomatic"))
{
  //mapping_manager(new OctreeManager(nh, tfBuffer, map_frame, ws_frame, tree_resolution, random_engine, robot_manager, 100, update_planning_tree, evaluation_mode)),7
  mapping_manager.reset(new OctreeManager(nh, priv_nh_, map_frame, tree_resolution));

  graph_manager.reset(new ViewposeGraphManager(robot_manager, mapping_manager, vt_searched_graph));

  // visualization topics
  roi_targets_pub = priv_nh_.advertise<sensor_msgs::PointCloud2>("vis_targets_roi", 1, true);
  expl_targets_pub = priv_nh_.advertise<sensor_msgs::PointCloud2>("vis_targets_expl", 1, true);
  border_targets_pub = priv_nh_.advertise<sensor_msgs::PointCloud2>("vis_targets_border", 1, true);

  // TODO
  workspacePub = nh.advertise<visualization_msgs::Marker>("workspace", 1, true);
  samplingRegionPub = nh.advertise<visualization_msgs::Marker>("samplingRegion", 1, true);
  config_server.setCallback(boost::bind(&ViewMotionPlanner::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2));
  vt_robot_state->loadMarkerPub(true);
  vt_robot_state->loadRobotStatePub("planned_state");
  vt_robot_state->setManualSceneUpdating();
}

ViewMotionPlanner::~ViewMotionPlanner()
{
  graph_builder_condition.shutdown();
  pose_visualizer_condition.shutdown();
  graph_visualizer_condition.shutdown();
  //graph_builder_condition.waitForShutdown();
  //pose_visualizer_condition.waitForShutdown();
  //graph_visualizer_condition.waitForShutdown();
  for (boost::thread &t : graph_builder_threads)
  {
    if (t.joinable())
      t.join();
  }
  if (pose_visualize_thread.joinable())
    pose_visualize_thread.join();
  if (graph_visualize_thread.joinable())
    graph_visualize_thread.join();
}

void ViewMotionPlanner::reconfigureCallback(VmpConfig &new_config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure callback called, level " << level);

  // Adjust sensor range if necessary
  if (level & (1 << 5)) // minimum sensor range
  {
    if (new_config.sensor_max_range < new_config.sensor_min_range)
      new_config.sensor_max_range = new_config.sensor_min_range;
  }
  else if (level & (1 << 6)) // maximum sensor range
  {
    if (new_config.sensor_min_range > new_config.sensor_max_range)
      new_config.sensor_min_range = new_config.sensor_max_range;
  }

  // Adjust target ratio if necessary
  double ratio_diff = 1 - (new_config.roi_target_ratio + new_config.expl_target_ratio + new_config.border_target_ratio);
  if (ratio_diff != 0.0)
  {
    if (level & (1 << 9)) // roi_target_ratio
    {
      double other_sum = new_config.expl_target_ratio + new_config.border_target_ratio;
      double expl_diff, border_diff;
      if (other_sum != 0.0)
      {
        expl_diff = ratio_diff * new_config.expl_target_ratio / other_sum;
        border_diff = ratio_diff * new_config.border_target_ratio / other_sum;
      }
      else
      {
        expl_diff = border_diff = 0.5 * ratio_diff;
      }
      new_config.expl_target_ratio += expl_diff;
      new_config.border_target_ratio += border_diff;
    }
    else if (level & (1 << 10)) // expl_target_ratio
    {
      new_config.border_target_ratio += ratio_diff;
      if (new_config.border_target_ratio < 0)
      {
        new_config.roi_target_ratio += new_config.border_target_ratio;
        new_config.border_target_ratio = 0;
      }
    }
    else if (level & (1 << 11)) // border_target_ratio
    {
      new_config.expl_target_ratio += ratio_diff;
      if (new_config.expl_target_ratio < 0)
      {
        new_config.roi_target_ratio += new_config.expl_target_ratio;
        new_config.expl_target_ratio = 0;
      }
    }
  }

  bool publish_workspace = false;
  bool publish_sampling_region = false;

  // Adjust workspace/sampling region if necessary
  if (level & (1 << 16)) // workspace minimum
  {
    if (new_config.ws_max_x < new_config.ws_min_x)
      new_config.ws_max_x = new_config.ws_min_x;
    if (new_config.ws_max_y < new_config.ws_min_y)
      new_config.ws_max_y = new_config.ws_min_y;
    if (new_config.ws_max_z < new_config.ws_min_z)
      new_config.ws_max_z = new_config.ws_min_z;

    publish_workspace = true;
  }
  else if (level & (1 << 17)) // workspace maximum
  {
    if (new_config.ws_min_x > new_config.ws_max_x)
      new_config.ws_min_x = new_config.ws_max_x;
    if (new_config.ws_min_y > new_config.ws_max_y)
      new_config.ws_min_y = new_config.ws_max_y;
    if (new_config.ws_min_z > new_config.ws_max_z)
      new_config.ws_min_z = new_config.ws_max_z;

    publish_workspace = true;
  }

  if (level & (1 << 18)) // sampling region minimum
  {
    if (new_config.sr_max_x < new_config.sr_min_x)
      new_config.sr_max_x = new_config.sr_min_x;
    if (new_config.sr_max_y < new_config.sr_min_y)
      new_config.sr_max_y = new_config.sr_min_y;
    if (new_config.sr_max_z < new_config.sr_min_z)
      new_config.sr_max_z = new_config.sr_min_z;

    publish_sampling_region = true;
  }
  else if (level & (1 << 19)) // sampling region maximum
  {
    if (new_config.sr_min_x > new_config.sr_max_x)
      new_config.sr_min_x = new_config.sr_max_x;
    if (new_config.sr_min_y > new_config.sr_max_y)
      new_config.sr_min_y = new_config.sr_max_y;
    if (new_config.sr_min_z > new_config.sr_max_z)
      new_config.sr_min_z = new_config.sr_max_z;

    publish_sampling_region = true;
  }

  config = new_config;

  if (publish_workspace) publishWorkspaceMarker();
  if (publish_sampling_region) publishSamplingRegionMarker();
}

void ViewMotionPlanner::publishWorkspaceMarker()
{
  visualization_msgs::Marker cube_msg;
  cube_msg.header.frame_id = ws_frame;
  cube_msg.header.stamp = ros::Time::now();
  cube_msg.ns = "workspace";
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::Marker::LINE_LIST;
  cube_msg.color = COLOR_RED;

  cube_msg.scale.x = 0.01;
  cube_msg.pose.orientation.w = 1.0; // normalize quaternion

  octomap::point3d min(config.ws_min_x, config.ws_min_y, config.ws_min_z);
  octomap::point3d max(config.ws_max_x, config.ws_max_y, config.ws_max_z);
  ROS_ERROR_STREAM("WS: " << min << max);
  addCubeEdges(min, max, cube_msg.points);

  workspacePub.publish(cube_msg);
}

void ViewMotionPlanner::publishSamplingRegionMarker()
{
  visualization_msgs::Marker cube_msg;
  cube_msg.header.frame_id = ws_frame;
  cube_msg.header.stamp = ros::Time::now();
  cube_msg.ns = "samplingRegion";
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::Marker::LINE_LIST;
  cube_msg.color = COLOR_GREEN;
  cube_msg.scale.x = 0.01;
  cube_msg.pose.orientation.w = 1.0; // normalize quaternion

  octomap::point3d min(config.sr_min_x, config.sr_min_y, config.sr_min_z);
  octomap::point3d max(config.sr_max_x, config.sr_max_y, config.sr_max_z);
  ROS_ERROR_STREAM("SR: " << min << max);
  addCubeEdges(min, max, cube_msg.points);

  samplingRegionPub.publish(cube_msg);
}

void ViewMotionPlanner::poseVisualizeThread()
{
  pose_visualizer_condition.confirm_start();
  for (ros::Rate rate(0.2); ros::ok() && !pose_visualizer_condition.isShutdown(); rate.sleep())
  {
    pose_visualizer_condition.wait();

    if (pose_visualizer_condition.isShutdown())
      break;

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
  pose_visualizer_condition.confirm_shutdown();
}

void ViewMotionPlanner::graphVisualizeThread()
{
  graph_visualizer_condition.confirm_start();
  for (ros::Rate rate(1); ros::ok() && !graph_visualizer_condition.isShutdown(); rate.sleep())
  {
    graph_visualizer_condition.wait();

    if (graph_visualizer_condition.isShutdown())
      break;

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
  graph_visualizer_condition.confirm_shutdown();
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
  graph_builder_condition.confirm_start();
  //static const size_t MAX_GRAPH_VERTICES = 10000;
  std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
  while(ros::ok() && !graph_builder_condition.isShutdown())
  {
    graph_builder_condition.wait();

    if (graph_builder_condition.isShutdown())
      break;

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

    //ROS_FATAL("ViewposePtr vp = sampleRandomViewPose(type); !!!!!!");
    //ViewposePtr vp = sampleRandomViewPose(type);
    ViewposePtr vp = sampleRandomViewPose(type, mapping_manager, map_frame, ws_frame, tfBuffer, robot_manager);
    if (vp)
    {
      boost::unique_lock lock(graph_manager->getGraphMutex());
      if (graph_manager->getNearestVertexDistanceByVpSimilarity(vp) < config.vpd_threshold) // VP is too similar to existing edge
      {
        continue;
      }
      Vertex v = graph_manager->addViewpose(vp);
      size_t connections = graph_manager->connectNeighbors(v, static_cast<size_t>(config.max_nb_connect_count), config.max_nb_connect_dist);
      if (config.discard_unconnected_vertices && connections == 0)
      {
        graph_manager->removeVertex(v);
      }
    }
  }
  graph_builder_condition.confirm_shutdown();
}

std::optional<Vertex> ViewMotionPlanner::initCameraPoseGraph()
{
  boost::unique_lock lock(graph_manager->getGraphMutex());
  ViewposePtr cam_vp(new Viewpose());
  cam_vp->state = robot_manager->getCurrentState();
  if (!cam_vp->state) return std::nullopt;
  cam_vp->pose = transform(robot_manager->getCurrentPose(), ws_frame, map_frame, tfBuffer);
  Vertex cam_vert = graph_manager->addViewpose(cam_vp);
  return cam_vert;
}

bool ViewMotionPlanner::buildGraph()
{
  resumeGraphBuilderThreads();
  std::optional<Vertex> cam_vert = initCameraPoseGraph();
  if (!cam_vert)
  {
    ROS_ERROR("Camera pose not initialized");
    return false;
  }
  graph_manager->initStartPose(*cam_vert);
  ros::Duration(config.graph_building_time).sleep();

  for (ros::Rate rate(1); ros::ok() && config.mode >= Vmp_BUILD_GRAPH; rate.sleep()) // connect start pose if not connected
  {
    boost::unique_lock lock(graph_manager->getGraphMutex());
    graph_manager->connectNeighbors(*cam_vert, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
    size_t num_out_edges = boost::out_degree(*cam_vert, graph_manager->getGraph());
    ROS_INFO_STREAM("Number of out edges from cam: " << num_out_edges);
    if (num_out_edges > 0)
      return true;
  }
  return false;
}

bool ViewMotionPlanner::searchPath()
{
  pauseGraphBuilderThreads();
  ros::Time start_expand(ros::Time::now());
  bool has_expanded = false;
  while (ros::ok())
  {
    bool success = graph_manager->expand();
    if (!success || (ros::Time::now() - start_expand).toSec() > config.graph_search_time)
      break;

    has_expanded = true;
  }
  if (config.visualize_graph && has_expanded)
    graph_manager->visualizeGraph(config.visualize_expanded, config.visualize_unexpanded);

  return has_expanded;
}

bool ViewMotionPlanner::executePath()
{
  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> trajectories = graph_manager->getNextTrajectories(config.execution_cost_limit);
  std::optional<Vertex> next_start_vertex = graph_manager->getCurrentStartVertex();
  bool moved = false;
  ROS_INFO_STREAM("Executing " << trajectories.size() << " trajectories");
  if (trajectories.size() == 0)
  {
    ROS_WARN_STREAM("No trajectories; resetting graph");
    graph_manager->clear();
    next_start_vertex = initCameraPoseGraph();
    if (!next_start_vertex)
    {
      ROS_ERROR("Camera pose not initialized");
    }
    return false;
  }
  for (const auto &[next_vertex, traj, cost] : trajectories)
  {
    ViewposePtr vp = graph_manager->getGraph()[next_vertex];
    if (!traj)
    {
      ROS_WARN_STREAM("No trajectory found");
      graph_manager->clear();
      next_start_vertex = initCameraPoseGraph();
      if (!next_start_vertex)
      {
        ROS_ERROR("Camera pose not initialized");
        return false;
      }
      graph_manager->connectNeighbors(*next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
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
      if (config.fix_trajectories_on_execution)
      {
        graph_manager->fixTrajectoryStartPoint(traj, robot_manager->getCurrentState());
      }
      bool success = robot_manager->executeTrajectory(traj);
      moved = true;
      graph_manager->markAsVisited(*next_start_vertex);
      waitForPointcloudWithRoi();
      if (evaluation_mode)
      {
        ROS_ERROR("Disabled evaluation code! Check line %d", __LINE__);
        //mapping_manager->saveEvaluatorData(cost, traj->getDuration());
      }
      if (!success)
      {
        ROS_WARN_STREAM("Failed to execute trajectory, setting new start pose");
        graph_manager->clear();
        next_start_vertex = initCameraPoseGraph();
        if (!next_start_vertex)
        {
          ROS_ERROR("Camera pose not initialized");
          return false;
        }
        graph_manager->connectNeighbors(*next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
        break;
      }
      else
      {
        //mapping_manager->updatePastViewposesList(vp);
        next_start_vertex = next_vertex;
      }
    }
  }
  if (moved)
  {
    graph_manager->cleanupAfterMove();
    graph_manager->initStartPose(*next_start_vertex);
  }
  return moved;
}

void ViewMotionPlanner::pathSearcherThread(EvalEpisodeEndParam ep, double duration)
{
  /*bool start_connected = */buildGraph();

  if (config.mode < Vmp_PLAN && config.insert_scan_if_not_moved)
    waitForPointcloudWithRoi();

  while (ros::ok() && config.mode >= Vmp_PLAN)
  {
    if (!searchPath() || config.mode < Vmp_PLAN_AND_EXECUTE)
    {
      if (config.insert_scan_if_not_moved)
        waitForPointcloudWithRoi();

      break;
    }

    bool moved = executePath();

    if (!moved && config.insert_scan_if_not_moved)
      waitForPointcloudWithRoi();

    // TODO: Rework
    // if ( (ep == EvalEpisodeEndParam::TIME && mapping_manager->getEvalPassedTime() > duration) ||
    //      (ep == EvalEpisodeEndParam::PLAN_DURATION && mapping_manager->getEvalAccPlanDuration() > duration) ||
    //      (ep == EvalEpisodeEndParam::PLAN_LENGTH && mapping_manager->getEvalAccPlanLength() > duration)
    //    )
    // {
    //   break;
    // }

    resumeGraphBuilderThreads();
    ros::Duration(config.graph_building_time).sleep();
  }
  pauseGraphBuilderThreads();
}

void ViewMotionPlanner::pathSearcherThread(const ros::Time &end_time)
{
  /*bool start_connected = */buildGraph();

  if (config.mode < Vmp_PLAN && config.insert_scan_if_not_moved)
    waitForPointcloudWithRoi();

  while (ros::ok() && config.mode >= Vmp_PLAN && ros::Time::now() < end_time)
  {
    if (!searchPath())
    {
      if (config.insert_scan_if_not_moved)
        waitForPointcloudWithRoi();

      break; // TODO: make configurable
    }
    
    if (config.mode < Vmp_PLAN_AND_EXECUTE)
      break;

    bool moved = executePath();

    if (!moved && config.insert_scan_if_not_moved)
      waitForPointcloudWithRoi();

    resumeGraphBuilderThreads();
    ros::Duration(config.graph_building_time).sleep();
  }
  pauseGraphBuilderThreads();
}

void ViewMotionPlanner::initGraphBuilderThreads()
{
  graph_builder_threads.reserve(NUM_GRAPH_BUILDER_THREADS);
  for (size_t i = 0; i < NUM_GRAPH_BUILDER_THREADS; i++)
  {
    graph_builder_threads.push_back(boost::move(boost::thread(boost::bind(&ViewMotionPlanner::graphBuilderThread, this))));
  }
}

void ViewMotionPlanner::pauseGraphBuilderThreads()
{
  graph_builder_condition.pause();
  graph_builder_condition.waitForPause();
}

void ViewMotionPlanner::resumeGraphBuilderThreads()
{
  graph_builder_condition.resume();
}

void ViewMotionPlanner::exploreNamedPoses()
{
  std::vector<std::string> pose_list;
  nh_.param<std::vector<std::string>>("view_motion_planner/pose_list", pose_list, {"explpose1", "explpose2", "explpose3", "explpose4", "explpose5", "home"});

  for (const std::string &pose : pose_list)
  {
    robot_manager->moveToNamedPose(pose);
    waitForPointcloudWithRoi();
  }
}

void ViewMotionPlanner::plannerLoop()
{    
  ROS_INFO_STREAM("PLANNER LOOP CALLED");
  waitForPointcloudWithRoi();
  pose_visualize_thread = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::poseVisualizeThread, this)));
  publish_map_thread = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::publishMapThread, this)));
  //graph_visualize_thread = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::graphVisualizeThread, this)));
  pose_visualizer_condition.resume();
  //graph_visualizer_condition.resume();

  initGraphBuilderThreads();

  if (evaluation_mode)
  {
    ROS_INFO_STREAM("EVALUATION MODE ACTIVATED");
    config.mode = Vmp_PLAN_AND_EXECUTE;
    updateConfig();

    ROS_ERROR("Disabled evaluation code! Check line %d", __LINE__);
    //mapping_manager->startEvaluator();

    for (size_t current_episode=0; ros::ok() && current_episode < eval_num_episodes; current_episode++)
    {
      pathSearcherThread(ep, eval_episode_duration);
      pauseGraphBuilderThreads();
      robot_manager->moveToHomePose();
      mapping_manager->resetMap();
      graph_manager->clear();
      waitForPointcloudWithRoi();

      ROS_ERROR("Disabled evaluation code! Check line %d", __LINE__);
      //mapping_manager->resetEvaluator();
    }
  }
  else
  {
    ROS_INFO_STREAM("ENTER MAIN LOOP");
    for (ros::Rate rate(100); ros::ok(); rate.sleep())
    {
      if (config.mode == Vmp_EXPLORE_NAMED_POSES)
      {
        ROS_INFO_STREAM("Explore named poses");
        exploreNamedPoses();
        config.mode = Vmp_IDLE;
        updateConfig();
      }
      else if (config.mode == Vmp_PLAN_WITH_TROLLEY)
      {
        //mapping_manager->clearPastViewposesList();
	if (config.trolley_plan_named_poses)
	{
          ROS_INFO_STREAM("Explore named poses for segment");
          exploreNamedPoses();
	}
        ROS_INFO_STREAM("Planning new segment");     
        graph_manager->clear();
        pathSearcherThread(ros::Time::now() + ros::Duration(config.trolley_time_per_segment));
        const std::string TROLLEY_TREE_PREFIX = "trolleyTree_segment";
        mapping_manager->saveToFile(generateFilename(TROLLEY_TREE_PREFIX + std::to_string(trolley_current_segment), true));
        graph_manager->clear();
        ROS_INFO_STREAM("Moving to home pose");
        robot_manager->moveToHomePose();
        trolley_current_segment++;
        if (trolley_current_segment >= config.trolley_num_segments) // end reached, go to idle
        {
          config.mode = Vmp_IDLE;
          updateConfig();
          continue;
        }
        ROS_INFO_STREAM("Moving trolley");
        trolley_remote.moveTo(static_cast<float>(trolley_remote.getPosition() + config.trolley_move_length));
        for (ros::Rate waitTrolley(10); ros::ok() && !trolley_remote.isReady(); waitTrolley.sleep());
        ros::Duration(5).sleep(); // wait for transform to update
        waitForPointcloudWithRoi();
        /*ROS_INFO_STREAM("Moving up");
        double h = trolley_remote.getHeight();
        ROS_INFO_STREAM("Height: " << h);
        const double LIFT = 500;
        static double sign = 1;
        trolley_remote.liftTo(sign * LIFT);
        ROS_INFO_STREAM("Lifting trolley");
        for (ros::Rate waitTrolley(10); ros::ok() && !trolley_remote.isReady(); waitTrolley.sleep());
        ROS_INFO_STREAM("Trolley Ready");
        config.mode = Vmp_IDLE;
        sign *= -1;
        updateConfig();*/
      }
      else if (config.mode >= Vmp_BUILD_GRAPH)
      {
        ROS_INFO_STREAM("PLANNER BUILD GRAPH AND PLAN");
        pathSearcherThread();
      }
      else if (config.mode == Vmp_MAP_ONLY)
      {
        ROS_INFO_STREAM("PLANNER MAPPING");
        waitForPointcloudWithRoi();
      }
      else
      {
        //ROS_INFO_STREAM("PLANNER IDLING");
      }
    }
  }

  /*for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }*/
}

void ViewMotionPlanner::waitForPointcloudWithRoi(double max_wait)
{
  pointcloud_roi_msgs::PointcloudWithRoiConstPtr msg = ros::topic::waitForMessage<pointcloud_roi_msgs::PointcloudWithRoi>("/detect_roi/results", nh_, ros::Duration(max_wait));
  
  geometry_msgs::Transform pc_transform;
  if (msg->cloud.header.frame_id != map_frame)
  {
    try
    {
      geometry_msgs::TransformStamped tmp_ = tfBuffer.lookupTransform(map_frame, msg->cloud.header.frame_id, msg->cloud.header.stamp);
      pc_transform = tmp_.transform;
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to map frame in registerPointcloudWithRoi: " << e.what());
      return;
    }
  }
  else
  {
    pc_transform.rotation.x = 0.0;
    pc_transform.rotation.y = 0.0;
    pc_transform.rotation.z = 0.0;
    pc_transform.rotation.w = 1.0;
    pc_transform.rotation.x = 0.0;
    pc_transform.rotation.y = 0.0;
    pc_transform.rotation.z = 0.0;
  }
  
  bool result = mapping_manager->registerPointcloudWithRoi(msg, pc_transform);
  if (result)
  {
    octomap::point3d sr_min_ws(config.sr_min_x, config.sr_min_y, config.sr_min_z);
    octomap::point3d sr_max_ws(config.sr_max_x, config.sr_max_y, config.sr_max_z);

    octomap::point3d sr_min_map = transform(sr_min_ws, ws_frame, map_frame, tfBuffer);
    octomap::point3d sr_max_map = transform(sr_max_ws, ws_frame, map_frame, tfBuffer);

    mapping_manager->updateTargets(sr_min_map, sr_max_map);

    // Publish targets
    if (roi_targets_pub.getNumSubscribers() > 0)
    {
      auto roi_targets = mapping_manager->getRoiTargets();
      auto roi_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZ>(*roi_targets, map_frame);
      //auto roi_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZRGB>(*roi_targets, map_frame, 255, 0 , 0); // RED
      roi_targets_pub.publish(roi_targets_msg);
    }
    if (expl_targets_pub.getNumSubscribers() > 0)
    {
      auto expl_targets = mapping_manager->getExplTargets();
      auto expl_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZ>(*expl_targets, map_frame);
      //auto expl_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZRGB>(*expl_targets, map_frame, 0, 255 , 0); // GREEN
      expl_targets_pub.publish(expl_targets_msg);
    }
    if (border_targets_pub.getNumSubscribers() > 0)
    {
      auto border_targets = mapping_manager->getBorderTargets();
      auto border_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZ>(*border_targets, map_frame);
      //auto border_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZRGB>(*border_targets, map_frame, 0, 0, 255); // BLUE
      border_targets_pub.publish(border_targets_msg);
    }

    //need_to_publish_map = need_to_publish_map || result;
  }
}

void ViewMotionPlanner::publishMapThread()
{
  ros::Rate r(0.2); // TODO: can be parametrized
  while (ros::ok())
  {
    //if (need_to_publish_map)
      mapping_manager->publishMap();
    r.sleep();
  }
  
}

} // namespace view_motion_planner
