#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer,
                                     const std::string &map_frame, const std::string &ws_frame, const std::string &pose_frame,
                                     const std::string &robot_description_param_name, const std::string &group_name, const std::string &ee_link_name,
                                     double tree_resolution, size_t graph_builder_threads, bool update_planning_tree, bool initialize_evaluator)
  : nh_(nh),
    random_engine(std::random_device{}()),
    map_frame(map_frame),
    ws_frame(ws_frame),
    pose_frame(pose_frame),
    NUM_GRAPH_BUILDER_THREADS(graph_builder_threads),
    update_planning_tree(update_planning_tree),
    eval_initialized(initialize_evaluator),
    eval_running(false),
    eval_start(false),
    config_server(config_mutex),
    vt_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_graph")),
    vt_searched_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_searched_graph")),
    robot_manager(new RobotManager(nh, tfBuffer, pose_frame, robot_description_param_name, group_name, ee_link_name)),
    vt_robot_state(new moveit_visual_tools::MoveItVisualTools(map_frame, "vm_robot_state", robot_manager->getPlanningSceneMonitor())),
    octree_manager(new OctreeManager(nh, tfBuffer, map_frame, ws_frame, pose_frame, tree_resolution, random_engine, robot_manager, 100, update_planning_tree, initialize_evaluator)),
    graph_manager(new ViewposeGraphManager(robot_manager, octree_manager, vt_searched_graph, map_frame, ws_frame, pose_frame)),
    trolley_remote(ros::NodeHandle(), ros::NodeHandle("/trollomatic"))
{
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
  //ROS_INFO_STREAM("WS: " << min << max);
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
  //ROS_INFO_STREAM("SR: " << min << max);
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

    ViewposePtr vp = octree_manager->sampleRandomViewPose(type);
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

void ViewMotionPlanner::computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells)
{
  octomap::pose6d viewpose = octree_manager->transform(robot_manager->getRobotStatePose(state), pose_frame, map_frame);
  octree_manager->computePoseObservedCells(viewpose, freeCells, occCells, unkCells);
}

std::optional<Vertex> ViewMotionPlanner::initCameraPoseGraph()
{
  boost::unique_lock lock(graph_manager->getGraphMutex());
  ViewposePtr cam_vp(new Viewpose());
  cam_vp->state = robot_manager->getCurrentState();
  if (!cam_vp->state) return std::nullopt;
  cam_vp->pose = octree_manager->transform(robot_manager->getCurrentPose(), pose_frame, map_frame);
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
  if (config.pause_graph_building_on_search)
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
  if (config.visualize_graph)
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
      octree_manager->waitForPointcloudWithRoi();
      if (eval_running)
      {
        octree_manager->saveEvaluatorData(cost, traj->getDuration(), eval_current_segment, trolley_remote.getPosition(), trolley_remote.getHeight());
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
        octree_manager->updatePastViewposesList(vp);
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
    octree_manager->waitForPointcloudWithRoi();

  while (ros::ok() && config.mode >= Vmp_PLAN)
  {
    if (!searchPath() || config.mode < Vmp_PLAN_AND_EXECUTE)
    {
      if (config.insert_scan_if_not_moved)
        octree_manager->waitForPointcloudWithRoi();

      break;
    }

    bool moved = executePath();

    if (!moved && config.insert_scan_if_not_moved)
      octree_manager->waitForPointcloudWithRoi();

    if ( (ep == EvalEpisodeEndParam::TIME && octree_manager->getEvalPassedTime() > duration) ||
         (ep == EvalEpisodeEndParam::PLAN_DURATION && octree_manager->getEvalAccPlanDuration() > duration) ||
         (ep == EvalEpisodeEndParam::PLAN_LENGTH && octree_manager->getEvalAccPlanLength() > duration)
       )
    {
      break;
    }

    resumeGraphBuilderThreads();
    if (config.pause_graph_building_on_search)
    {
      ros::Duration(config.graph_building_time).sleep();
    }
  }
  pauseGraphBuilderThreads();
}

void ViewMotionPlanner::pathSearcherThread(const ros::Time &end_time)
{
  /*bool start_connected = */buildGraph();

  if (config.mode < Vmp_PLAN && config.insert_scan_if_not_moved)
    octree_manager->waitForPointcloudWithRoi();

  while (ros::ok() && config.mode >= Vmp_PLAN && ros::Time::now() < end_time)
  {
    if (!searchPath())
    {
      if (config.insert_scan_if_not_moved)
        octree_manager->waitForPointcloudWithRoi();

      break; // TODO: make configurable
    }
    
    if (config.mode < Vmp_PLAN_AND_EXECUTE)
      break;

    bool moved = executePath();

    if (!moved && config.insert_scan_if_not_moved)
      octree_manager->waitForPointcloudWithRoi();

    resumeGraphBuilderThreads();
    if (config.pause_graph_building_on_search)
    {
      ros::Duration(config.graph_building_time).sleep();
    }
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
    octree_manager->waitForPointcloudWithRoi();
  }
}

bool ViewMotionPlanner::startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                    bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist,
                    bool with_trolley)
{
  if (!eval_initialized || eval_running || eval_start) // Evaluation not initialized, alread in progress or about to start
    return false;

  eval_num_episodes = start_index + numEvals;
  eval_current_episode = start_index;
  eval_episode_duration = episodeDuration;
  eval_epend_param = episodeEndParam;
  eval_with_trolley = with_trolley;
  eval_current_segment = 0;

  bool success = octree_manager->startEvaluator(numEvals, episodeEndParam, episodeDuration, start_index,
    randomize_plants, min, max, min_dist, with_trolley);

  if (!success) return false;

  eval_start = true;
  return true;
}

void ViewMotionPlanner::flipWsAndSr()
{
  config.ws_min_y = -(config.ws_min_y - 0.3);
  config.ws_max_y = -(config.ws_max_y - 0.3);
  std::swap(config.ws_min_y, config.ws_max_y);
  config.sr_min_y = -(config.sr_min_y - 0.3);
  config.sr_max_y = -(config.sr_max_y - 0.3);
  std::swap(config.sr_min_y, config.sr_max_y);

  updateConfig();

  robot_manager->flipHomePose();

  publishWorkspaceMarker();
  publishSamplingRegionMarker();
}

bool ViewMotionPlanner::trolleyGoNextSegment()
{
  robot_manager->moveToHomePose(); // move arm to home pose before moving to next segment

  if (config.trolley_flip_workspace && !trolley_current_flipped)
  {
    flipWsAndSr();
    robot_manager->moveToHomePose();
    trolley_current_flipped = true;
    eval_current_segment++;
    return true;
  }

  if (config.trolley_num_vertical_segments > 1 && trolley_current_vertical_segment < config.trolley_num_vertical_segments - 1)
  {
    trolley_current_vertical_segment++;
    ROS_INFO_STREAM("Lifting trolley");
    trolley_remote.liftTo(469.0 + trolley_current_vertical_segment * config.trolley_lift_dist);
    trolley_remote.waitForReady();
    ros::Duration(5).sleep(); // wait for transform to update
    if (config.trolley_flip_workspace && trolley_current_flipped)
    {
      flipWsAndSr();
      trolley_current_flipped = false;
    }
    robot_manager->moveToHomePose();
    eval_current_segment++;
    return true;
  }

  if (config.trolley_num_segments > 1 && trolley_current_segment < config.trolley_num_segments - 1)
  {
    trolley_current_vertical_segment = 0;
    trolley_current_segment++;
    ROS_INFO_STREAM("Moving trolley");
    trolley_remote.moveTo(static_cast<float>(trolley_current_segment * config.trolley_move_length));
    trolley_remote.waitForReady();
    trolley_remote.liftTo(469.0);
    trolley_remote.waitForReady();
    ros::Duration(5).sleep(); // wait for transform to update
    if (config.trolley_flip_workspace && trolley_current_flipped)
    {
      flipWsAndSr();
      trolley_current_flipped = false;
    }
    robot_manager->moveToHomePose();
    eval_current_segment++;
    return true;
  }

  return false;
}

void ViewMotionPlanner::plannerLoop()
{    
  ROS_INFO_STREAM("PLANNER LOOP CALLED");
  octree_manager->waitForPointcloudWithRoi();
  pose_visualize_thread = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::poseVisualizeThread, this)));
  //graph_visualize_thread = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::graphVisualizeThread, this)));
  pose_visualizer_condition.resume();
  //graph_visualizer_condition.resume();

  initGraphBuilderThreads();

  /*if (evaluation_mode)
  {
    ROS_INFO_STREAM("EVALUATION MODE ACTIVATED");
    config.mode = Vmp_PLAN_AND_EXECUTE;
    updateConfig();
    octree_manager->startEvaluator();
    for (size_t current_episode=0; ros::ok() && current_episode < eval_num_episodes; current_episode++)
    {
      pathSearcherThread(eval_epend_param, eval_episode_duration);
      pauseGraphBuilderThreads();
      robot_manager->moveToHomePose();
      octree_manager->resetOctomap();
      graph_manager->clear();
      octree_manager->waitForPointcloudWithRoi();
      octree_manager->resetEvaluator();
    }
  }*/
  ROS_INFO_STREAM("ENTER MAIN LOOP");
  for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    if (eval_start)
    {
      ROS_INFO_STREAM("STARTING_EVALUATION");
      assert(!eval_running);
      if (eval_with_trolley)
      {
        config.trolley_segment_end_param = static_cast<int>(eval_epend_param);
        config.trolley_time_per_segment = static_cast<int>(eval_episode_duration);
        config.mode = Vmp_PLAN_WITH_TROLLEY;
      }
      else
      {
        config.mode = Vmp_PLAN_AND_EXECUTE;
      }

      updateConfig();
      eval_running = true;
      eval_start = false;
    }

    if (config.mode == Vmp_EXPLORE_NAMED_POSES)
    {
      ROS_INFO_STREAM("Explore named poses");
      exploreNamedPoses();
      config.mode = Vmp_IDLE;
      updateConfig();
    }
    else if (config.mode == Vmp_PLAN_WITH_TROLLEY)
    {
      octree_manager->clearPastViewposesList();
      if (config.trolley_plan_named_poses)
      {
        ROS_INFO_STREAM("Explore named poses for segment");
        exploreNamedPoses();
      }
      ROS_INFO_STREAM("Planning new segment");
      graph_manager->clear();
      switch(config.trolley_segment_end_param)
      {
      case Vmp_TIME:
        pathSearcherThread(ros::Time::now() + ros::Duration(config.trolley_time_per_segment));
        break;
      case Vmp_PLAN_DURATION:
        pathSearcherThread(EvalEpisodeEndParam::PLAN_DURATION, (1 + eval_current_segment) * config.trolley_time_per_segment);
        break;
      case Vmp_PLAN_LENGTH:
        pathSearcherThread(EvalEpisodeEndParam::PLAN_LENGTH,  (1 + eval_current_segment) * config.trolley_time_per_segment);
        break;
      default:
        ROS_ERROR_STREAM("Invalid trolley segment end param: " << config.trolley_segment_end_param);
      }

      const std::string TROLLEY_TREE_PREFIX = "trolleyTree_segment";
      octree_manager->saveOctomap(TROLLEY_TREE_PREFIX + std::to_string(trolley_current_segment), true);
      graph_manager->clear();
      ROS_INFO_STREAM("Moving to home pose");
      robot_manager->moveToHomePose();
      if (!trolleyGoNextSegment()) // end reached, go to idle
      {
        if (eval_current_episode + 1 >= eval_num_episodes)
        {
          config.mode = Vmp_IDLE;
          updateConfig();
          continue;
        }
        robot_manager->moveToHomePose();
        trolley_remote.moveTo(0);
        trolley_remote.waitForReady();
        trolley_remote.liftTo(469.0);
        trolley_remote.waitForReady();
        ros::Duration(5).sleep(); // wait for transform to update

        // reset current segments
        trolley_current_segment = 0;
        trolley_current_vertical_segment = 0;
        eval_current_segment = 0;
        if (config.trolley_flip_workspace && trolley_current_flipped)
        {
          flipWsAndSr();
          trolley_current_flipped = false;
        }

        robot_manager->moveToHomePose();
        octree_manager->resetOctomap();
        graph_manager->clear();
        octree_manager->waitForPointcloudWithRoi();
        octree_manager->resetEvaluator();
        eval_current_episode++;
      }
      octree_manager->waitForPointcloudWithRoi();
      /*ROS_INFO_STREAM("Moving up");
      double h = trolley_remote.getHeight();
      ROS_INFO_STREAM("Height: " << h);
      const double LIFT = 500;
      static double sign = 1;
      trolley_remote.liftTo(sign * LIFT);
      ROS_INFO_STREAM("Lifting trolley");
      trolley_remote.waitForReady();
      ROS_INFO_STREAM("Trolley Ready");
      config.mode = Vmp_IDLE;
      sign *= -1;
      updateConfig();*/
    }
    else if (config.mode >= Vmp_BUILD_GRAPH)
    {
      ROS_INFO_STREAM("PLANNER BUILD GRAPH AND PLAN");
      if (eval_running)
      {
        pathSearcherThread(eval_epend_param, eval_episode_duration);
        pauseGraphBuilderThreads();
        robot_manager->moveToHomePose();
        octree_manager->resetOctomap();
        graph_manager->clear();
        octree_manager->waitForPointcloudWithRoi();
        octree_manager->resetEvaluator();
        eval_current_episode++;
        if (eval_current_episode >= eval_num_episodes)
        {
          eval_running = false;
          config.mode = Vmp_IDLE;
          updateConfig();
        }
      }
      else
      {

        pathSearcherThread();
      }
    }
    else if (config.mode == Vmp_MAP_ONLY)
    {
      ROS_INFO_STREAM("PLANNER MAPPING");
      octree_manager->waitForPointcloudWithRoi();
    }
    else
    {
      //ROS_INFO_STREAM("PLANNER IDLING");
    }
  }

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
