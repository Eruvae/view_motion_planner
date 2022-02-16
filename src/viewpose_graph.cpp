#include "view_motion_planner/viewpose_graph.h"

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
//#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/datastructures/NearestNeighborsLinear.h>
//#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

namespace view_motion_planner
{

bool VertexUtilityComp::operator() (const Vertex &v1, const Vertex &v2) const
{
  const ViewposePtr &vp1 = graph_manager->getGraph()[v1];
  const ViewposePtr &vp2 = graph_manager->getGraph()[v2];
  return vp1->accumulated_cost < vp2->accumulated_cost;
}

ViewposeGraphManager::ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager,
                                           const std::shared_ptr<OctreeManager> &octree_manager,
                                           const rviz_visual_tools::RvizVisualToolsPtr &vt_searched_graph)
  : robot_manager(robot_manager), octree_manager(octree_manager), neighbor_data(new ompl::NearestNeighborsGNAT<Vertex>()),
    vt_searched_graph(vt_searched_graph), current_start_vertex_number(0), priorityQueue(VertexUtilityComp(this))
{
  neighbor_data->setDistanceFunction(boost::bind(&ViewposeGraphManager::getVertexDistanceJoints, this, _1, _2));
}

double ViewposeGraphManager::getVertexDistancePose(Vertex a, Vertex b)
{
  const geometry_msgs::Point &pa = graph[a]->pose.position;
  const geometry_msgs::Point &pb = graph[b]->pose.position;
  double x = pb.x - pa.x;
  double y = pb.y - pa.y;
  double z = pb.z - pa.z;
  return x*x + y*y + z*z;
}

double ViewposeGraphManager::getVertexDistanceJoints(Vertex a, Vertex b)
{
  return graph[a]->state->distance(*(graph[b]->state), robot_manager->getJointModelGroup());
}

void ViewposeGraphManager::connectNeighbors(const Vertex &v, size_t num_neighbors, double max_traj_length, double traj_step)
{
  std::vector<Vertex> neighbors;
  neighbor_data->nearestK(v, num_neighbors, neighbors);

  for (const Vertex &nv : neighbors)
  {
    std::pair<Edge, bool> check_edge = boost::edge(v, nv, graph);
    if (check_edge.second) // edge already exists
      continue;

    moveit::core::RobotStatePtr from = graph[v]->state;
    moveit::core::RobotStatePtr to = graph[nv]->state;

    TrajectoryPtr t(new Trajectory());
    t->cost = from->distance(*to, robot_manager->getJointModelGroup());
    if (t->cost > max_traj_length) // don't compute trajectory
      continue;

    t->traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
    t->bw_traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
    t->traj->addSuffixWayPoint(from, 0);
    t->bw_traj->addPrefixWayPoint(from, 0);
    bool found_traj = true;
    if (t->cost > traj_step) // interpolate points
    {
      size_t num_steps = static_cast<size_t>(t->cost / traj_step);
      double frac_step = 1.0 / num_steps;
      for (size_t i = 1; i < num_steps; i++) // don't check first and last state (assume valid)
      {
        moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*from));
        from->interpolate(*to, i * frac_step, *temp_state, robot_manager->getJointModelGroup());
        temp_state->update();

        bool state_valid = robot_manager->isValidState(temp_state);
        if (!state_valid) // don't add edge
        {
          found_traj = false;
          break;
        }
        t->traj->addSuffixWayPoint(temp_state, 0);
        t->bw_traj->addPrefixWayPoint(temp_state, 0);
      }
    }
    if (found_traj)
    {
      t->traj->addSuffixWayPoint(to, 0);
      t->bw_traj->addPrefixWayPoint(to, 0);

      trajectory_processing::IterativeSplineParameterization trajectory_processor(true);
      found_traj = (trajectory_processor.computeTimeStamps(*(t->traj), 1.0, 1.0) && trajectory_processor.computeTimeStamps(*(t->bw_traj), 1.0, 1.0));
      if (!found_traj)
        continue;

      /*auto [edge, success] = */boost::add_edge(v, nv, t, graph);
      t->cost = std::accumulate(t->traj->getWayPointDurations().begin(), t->traj->getWayPointDurations().end(), 0);
      t->last_collision_check_start_vertex = current_start_vertex_number;
    }
  }
}

void ViewposeGraphManager::visualizeGraph()
{
  ROS_INFO_STREAM("Visualizing searched graph");
  vt_searched_graph->deleteAllMarkers();
  boost::shared_lock lock(graph_mtx);
  std::queue<Vertex> toVisualizeQueue;
  std::unordered_set<ViewposePtr> bestUtilityPoses;
  std::unordered_set<Vertex> visualizedSet;
  toVisualizeQueue.push(current_start_vertex);
  ROS_INFO_STREAM("Computing highest utility path");
  for (ViewposePtr vp = highest_ig_pose; vp != nullptr; vp=vp->pred)
  {
    bestUtilityPoses.insert(vp);
  }
  while(!toVisualizeQueue.empty())
  {
    ROS_INFO_STREAM("Visualize queue: " << toVisualizeQueue.size());
    Vertex v = toVisualizeQueue.front();
    toVisualizeQueue.pop();
    visualizedSet.insert(v);
    ViewposePtr vp = graph[v];
    for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
    {
      ViewposePtr target = graph[ei->m_target];
      if (visualizedSet.count(ei->m_target) > 0) // already visualized
        continue;

      rviz_visual_tools::colors line_color = rviz_visual_tools::DEFAULT;
      if (bestUtilityPoses.count(target) > 0) // is on best utility path
        line_color = rviz_visual_tools::CYAN;
      else if (target->pred != nullptr) // has been expanded
        line_color = rviz_visual_tools::DARK_GREY;
      else // not yet expanded
        line_color = rviz_visual_tools::GREY;

      vt_searched_graph->publishLine(graph[ei->m_source]->pose.position, graph[ei->m_target]->pose.position, line_color);
      toVisualizeQueue.push(ei->m_target);
    }
  }
  ROS_INFO_STREAM("Visualization done");
  vt_searched_graph->trigger();
}

void ViewposeGraphManager::initStartPose(const Vertex &v)
{
  current_start_vertex = v;
  priorityQueue.push(v);
  expanded_vertices.insert(v);
}

void ViewposeGraphManager::cleanupAfterMove(const Vertex &new_start_vertex)
{
  boost::unique_lock lock(graph_mtx);
  visited_vertices.insert(current_start_vertex);
  priorityQueue.clear();
  current_start_vertex = new_start_vertex;
  current_start_vertex_number++;
  for (const Vertex &v : expanded_vertices)
  {
    if (graph[v])
      graph[v]->clearPredecessor();
  }
  expanded_vertices.clear();
  current_start_vertex = new_start_vertex;
  priorityQueue.push(new_start_vertex);
  expanded_vertices.insert(new_start_vertex);
}

bool ViewposeGraphManager::expand()
{
  if (priorityQueue.empty())
    return false;

  boost::unique_lock lock(graph_mtx);

  Vertex v = priorityQueue.top();
  ViewposePtr vp = graph[v];
  priorityQueue.pop();

  moveit::core::RobotStatePtr cur_state = vp->state;
  std::vector<Edge> edges_to_remove;
  for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
  {

    TrajectoryPtr t = graph[*ei];
    if (t->last_collision_check_start_vertex != current_start_vertex_number) // last collision check performed before move
    {
      const robot_trajectory::RobotTrajectoryPtr &traj = cur_state == t->traj->getFirstWayPointPtr() ? t->traj : t->bw_traj;
      if (!robot_manager->isValidTrajectory(traj))
      {
        edges_to_remove.push_back(*ei);
        continue;
      }
      t->last_collision_check_start_vertex = current_start_vertex_number;
    }

    ViewposePtr target = graph[ei->m_target];
    if (ei->m_target == current_start_vertex || visited_vertices.count(ei->m_target) > 0) // not a valid vertex to visit
    {
      continue;
    }
    if (target->pred) // already visited before
    {
      // TODO: check if improved path
      continue;
    }
    target->addPredecessor(vp, t);
    octree_manager->computePoseObservedCells(octomap_vpp::poseToOctomath(target->pose), target->freeCells, target->occCells, target->unkCells);
    target->computeUtility();
    if (!highest_ig_pose || target->accumulated_infogain > highest_ig_pose->accumulated_infogain)
    {
      highest_ig_pose = target;
    }
    if (!highest_util_pose || target->accumulated_utility > highest_util_pose->accumulated_utility)
    {
      highest_util_pose = target;
    }
    //ValueHeap::handle_type handle =
    priorityQueue.push(ei->m_target);
    expanded_vertices.insert(ei->m_target);
  }
  for (const Edge &e : edges_to_remove)
  {
    boost::remove_edge(e, graph);
  }
  return true;
}

const std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr> ViewposeGraphManager::getNextTrajectory()
{
  boost::shared_lock lock(graph_mtx);
  ViewposePtr vp = highest_ig_pose;
  if (vp)
  {
    while(vp->pred && vp->pred->pred != nullptr)
    {
      vp = vp->pred;
    }
  }
  TrajectoryPtr t = vp->pred_edge;
  if (!t)
  {
    return {vertex_map[vp], nullptr};
  }
  robot_trajectory::RobotTrajectoryPtr traj = vp->pred->state == t->traj->getFirstWayPointPtr() ? t->traj : t->bw_traj;
  return {vertex_map[vp], traj};
}

} // namespace view_motion_planner
