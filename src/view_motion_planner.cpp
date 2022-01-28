#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                  const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator)
  : random_engine(std::random_device{}()),
    robot_manager(new RobotManager(nh, tfBuffer, map_frame)),
    octree_manager(new OctreeManager(nh, tfBuffer, wstree_file, sampling_tree_file, map_frame, ws_frame, tree_resolution, random_engine, robot_manager, 100, initialize_evaluator)),
    graph_manager(robot_manager),
    vt_robot_state(new moveit_visual_tools::MoveItVisualTools(map_frame, "vm_robot_state", robot_manager->getPlanningSceneMonitor())),
    vt_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_graph"))
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
      const Viewpose &vp = observationPoses[i];

      vt_robot_state->publishRobotState(vp.state);

      //octomap::pose6d viewpose = robot_manager->getRobotStatePose(vp.state);
      //geometry_msgs::Point vpm = octomap::pointOctomapToMsg(viewpose.trans());
      //octomap::point3d_collection endpoints = computeVpRaycastEndpoints(viewpose);

      octomap::pose6d viewpose = octomap_vpp::poseToOctomath(vp.pose);
      geometry_msgs::Point vpm = vp.pose.position;
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
    vt_graph->deleteAllMarkers();
    for (auto [ei, ei_end] = boost::edges(graph_manager.getGraph()); ei != ei_end; ei++)
    {
      vt_graph->publishLine(graph_manager.getGraph()[ei->m_source].pose.position, graph_manager.getGraph()[ei->m_target].pose.position, rviz_visual_tools::RvizVisualTools::intToRvizColor(component_map[ei->m_source] % 15));
      // visual_tools_->publishTrajectoryLine(graph_manager.getGraph()[*ei].traj, robot_manager->getJointModelGroup(), visual_tools_->intToRvizColor(component_map[ei->m_source] % 15));
    }
    graph_manager.getGraphMutex().unlock_shared();
    vt_graph->trigger();
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
  static const size_t MAX_GRAPH_VERTICES = 10000;
  std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
  while(ros::ok())
  {
    if(graph_manager.getGraph().m_vertices.size() > MAX_GRAPH_VERTICES)
      break;

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

octomap::point3d_collection ViewMotionPlanner::computeVpRaycastEndpoints(const octomap::pose6d &vp)
{
  octomap::point3d_collection endpoints;
  const double hfov = 80 * M_PI / 180.0;
  const double vfov = 60 * M_PI / 180.0;
  const size_t x_steps = 9;
  const size_t y_steps = 7;
  const double maxRange = 1.0;

  for (size_t i = 0; i < x_steps; i++)
  {
    double ha = -hfov/2 + (i / static_cast<double>(x_steps-1)) * hfov;
    for(size_t j = 0; j < y_steps; j++)
    {
      double va = -vfov/2 + (j / static_cast<double>(y_steps-1)) * vfov;
      octomap::point3d dir(1.0, tan(ha), tan(va));
      octomap::point3d end = dir * maxRange;
      endpoints.push_back(vp.transform(end));
    }
  }
  return endpoints;
}

void ViewMotionPlanner::computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells)
{
  octomap::pose6d viewpose = robot_manager->getRobotStatePose(state);
  octomap::point3d_collection endpoints = computeVpRaycastEndpoints(viewpose);
  for (const octomap::point3d &end : endpoints)
  {
    octree_manager->computeRayCells(viewpose.trans(), end, freeCells, occCells, unkCells);
  }
}

void ViewMotionPlanner::pathSearcherThread()
{
  struct VisitedVertex
  {
    VisitedVertex(const Vertex &v, const Viewpose &vp) : v(v), vp(vp), cost(0), utility(0) {}
    VisitedVertex(const Vertex &v, const Viewpose &vp, const VisitedVertex &pred, double cost) : v(v), vp(vp), cost(pred.cost + cost)
    {
      freeCells = pred.freeCells;
      occCells = pred.occCells;
      unkCells = pred.unkCells;
      computeUtility();
    }

    double computeUtility()
    {
      utility = unkCells.size() / cost;
    }

    Vertex v;
    const Viewpose &vp;
    double cost;
    double utility;
    octomap::KeySet freeCells;
    octomap::KeySet occCells;
    octomap::KeySet unkCells;

    bool operator< (const VisitedVertex &rhs) const
    {
      return utility < rhs.utility;
    }
  };

  typedef boost::heap::fibonacci_heap<VisitedVertex> ValueHeap;
  typedef std::unordered_map<Vertex, ValueHeap::handle_type> VertexHandleMap;

  ValueHeap priorityQueue;
  VertexHandleMap openVertices;
  std::unordered_set<Vertex> processedVertices;

  while(ros::ok())
  {
    graph_manager.getGraphMutex().lock();
    Viewpose cam_vp;
    cam_vp.state = robot_manager->getCurrentState();
    cam_vp.pose = robot_manager->getCurrentPose();
    Vertex cam_vert = graph_manager.addViewpose(cam_vp);
    graph_manager.connectNeighbors(cam_vert, 5, DBL_MAX);
    graph_manager.getGraphMutex().unlock();

    ValueHeap::handle_type handle = priorityQueue.push(VisitedVertex(cam_vert, cam_vp));
    openVertices[cam_vert] = handle;

    for (ros::Rate rate(1); ros::ok(); rate.sleep())
    {
      const VisitedVertex &vert = priorityQueue.top();
      priorityQueue.pop();

      graph_manager.getGraphMutex().lock();
      moveit::core::RobotStatePtr cur_state = vert.vp.state;
      for (auto [ei, ei_end] = boost::out_edges(vert.v, graph_manager.getGraph()); ei != ei_end; ei++)
      {

        const Trajectory &t = graph_manager.getGraph()[*ei];
        const robot_trajectory::RobotTrajectoryPtr &traj = cur_state == t.traj->getFirstWayPointPtr() ? t.traj : t.bw_traj;

        const Viewpose &target = graph_manager.getGraph()[ei->m_target];
        VisitedVertex target_vert(ei->m_target, target, vert, t.cost);
        computeStateObservedVoxels(target.state, target_vert.freeCells, target_vert.occCells, target_vert.unkCells);
        ValueHeap::handle_type handle = priorityQueue.push(target_vert);
        //robot_manager->executeTrajectory(traj);
      }
      graph_manager.getGraphMutex().unlock();
    }
  }
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
  boost::thread pathSearcherThread(boost::bind(&ViewMotionPlanner::pathSearcherThread, this));
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
