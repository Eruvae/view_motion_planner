#include "view_motion_planner/viewpose_graph.h"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
//#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/datastructures/NearestNeighborsLinear.h>
//#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <view_motion_planner/vmp_utils.h>

namespace view_motion_planner
{

bool VertexUtilityComp::operator() (const Vertex &v1, const Vertex &v2) const
{
  const ViewposePtr &vp1 = graph_manager->getGraph()[v1];
  const ViewposePtr &vp2 = graph_manager->getGraph()[v2];
  return vp1->accumulated_cost < vp2->accumulated_cost;
}

ViewposeGraphManager::ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager,
                                           const std::shared_ptr<BaseMappingManager> &mapping_manager,
                                           const rviz_visual_tools::RvizVisualToolsPtr &vt_searched_graph,
                                           const std::string &map_frame, const std::string &ws_frame, const std::string &pose_frame)
  : robot_manager(robot_manager), mapping_manager(mapping_manager),
    neighbors_joint(new ompl::NearestNeighborsGNAT<ViewposePtr>()), neighbors_vpsim(new ompl::NearestNeighborsGNAT<ViewposePtr>()),
    vt_searched_graph(vt_searched_graph), current_start_vertex_number(0), priorityQueue(VertexUtilityComp(this)),
    map_frame(map_frame), ws_frame(ws_frame), pose_frame(pose_frame)
{
  neighbors_joint->setDistanceFunction(std::bind(&ViewposeGraphManager::getVertexDistanceJoints, this, std::placeholders::_1, std::placeholders::_2));
  neighbors_vpsim->setDistanceFunction(std::bind(&ViewposeGraphManager::getVertexDistanceVpSimilarity, this, std::placeholders::_1, std::placeholders::_2));

  ros::NodeHandle nh;
  viewArrowVisPub = nh.advertise<visualization_msgs::MarkerArray>("roi_vp_marker", 1);
  poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("vp_array", 1);
}

double ViewposeGraphManager::getVertexDistancePose(ViewposePtr a, ViewposePtr b)
{
  const geometry_msgs::Point &pa = a->pose.position;
  const geometry_msgs::Point &pb = b->pose.position;
  double x = pb.x - pa.x;
  double y = pb.y - pa.y;
  double z = pb.z - pa.z;
  return x*x + y*y + z*z;
}

double ViewposeGraphManager::getVertexDistanceJoints(ViewposePtr a, ViewposePtr b)
{
  return a->state->distance(*(b->state), robot_manager->getJointModelGroup());
}

double ViewposeGraphManager::getVertexDistanceVpSimilarity(ViewposePtr a, ViewposePtr b)
{
  return b->computeViewpointDissimilarity(a); // return computeViewpointDissimilarity(a, b);
}

double ViewposeGraphManager::getNearestVertexDistanceByVpSimilarity(const ViewposePtr &vp)
{
  ViewposePtr nb = neighbors_vpsim->nearest(vp);
  return neighbors_vpsim->getDistanceFunction()(vp, nb);
}

size_t ViewposeGraphManager::connectNeighbors(const Vertex &v, size_t num_neighbors, double max_traj_length, double traj_step)
{
  size_t successful_connections = 0;
  std::vector<ViewposePtr> neighbors;
  neighbors_joint->nearestK(graph[v], num_neighbors, neighbors);

  for (const ViewposePtr &nb : neighbors)
  {
    Vertex nv = vertex_map.at(nb);
    std::pair<Edge, bool> check_edge = boost::edge(v, nv, graph);
    if (check_edge.second) // edge already exists
      continue;

    moveit::core::RobotStatePtr from = graph[v]->state;
    moveit::core::RobotStatePtr to = nb->state;

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

      trajectory_processing::TimeOptimalTrajectoryGeneration trajectory_processor;
      found_traj = (trajectory_processor.computeTimeStamps(*(t->traj), 1.0, 1.0) && trajectory_processor.computeTimeStamps(*(t->bw_traj), 1.0, 1.0));
      if (!found_traj)
        continue;

      t->traj_start_state = from;
      t->bw_traj_start_state = to;

      /*auto [edge, success] = */boost::add_edge(v, nv, t, graph);
      //t->cost = std::accumulate(t->traj->getWayPointDurations().begin(), t->traj->getWayPointDurations().end(), 0);
      t->last_collision_check_start_vertex = current_start_vertex_number;
      successful_connections++;
    }
  }
  return successful_connections;
}

void ViewposeGraphManager::publishViewposeVisualizations(const std::vector<ViewposePtr> &vps)
{
  static size_t last_marker_count = 0;
  visualization_msgs::MarkerArray markers;
  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = map_frame;
  poseArr.header.stamp = ros::Time::now();
  bool first = true;
  std::string ns = "graphPoses";
  for (size_t i = 0; i < vps.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    switch (vps[i]->type)
    {
    case TARGET_ROI:
      marker.color = COLOR_RED;
      break;
    case TARGET_OCC:
      marker.color = COLOR_GREEN;
      break;
    case TARGET_BORDER:
      marker.color = COLOR_BLUE;
      break;
    }
    //marker.lifetime = ros::Duration(2);
    marker.points.push_back(vps[i]->pose.position);
    marker.points.push_back(octomap::pointOctomapToMsg(vps[i]->origin));
    markers.markers.push_back(marker);

    /*visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.pose.position = viewpoints[i].pose.position;
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.z = 0.05;
    textMarker.color = color;
    //textMarker.lifetime = ros::Duration(2);
    textMarker.text = std::to_string(viewpoints[i].infoGain) + ", " + std::to_string(viewpoints[i].distance) + ", " + std::to_string(viewpoints[i].utility);
    markers.markers.push_back(textMarker);*/

    poseArr.poses.push_back(vps[i]->pose);
  }
  ROS_INFO_STREAM("Last Marker count namespace: " << ns << ": " << last_marker_count << "; current: " << vps.size());
  for (size_t i = vps.size(); i < last_marker_count; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);
    /*visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(textMarker);*/
  }
  last_marker_count = vps.size();
  viewArrowVisPub.publish(markers);
  poseArrayPub.publish(poseArr);
}

void ViewposeGraphManager::visualizeGraph(bool visualize_expanded, bool visualize_unexpanded)
{
  std::vector<ViewposePtr> vps;
  ROS_INFO_STREAM("Visualizing searched graph");
  vt_searched_graph->deleteAllMarkers();
  boost::shared_lock lock(graph_mtx);
  std::queue<Vertex> toVisualizeQueue;
  std::unordered_set<ViewposePtr> bestUtilityPoses;
  std::unordered_set<Vertex> visualizedSet;
  toVisualizeQueue.push(current_start_vertex);
  //ROS_INFO_STREAM("Computing highest utility path");
  for (ViewposePtr vp = config.goal_select_type == Vmp_BEST_UTILITY ? highest_util_pose : highest_ig_pose; vp != nullptr && ros::ok(); vp=vp->pred)
  {
    bestUtilityPoses.insert(vp);
  }
  size_t verts_bu = 0, verts_vis = 0, verts_exp = 0, verts_ue = 0;
  while(!toVisualizeQueue.empty() && ros::ok())
  {
    //ROS_INFO_STREAM("Visualize queue: " << toVisualizeQueue.size());
    Vertex v = toVisualizeQueue.front();
    toVisualizeQueue.pop();
    visualizedSet.insert(v);
    ViewposePtr vp = graph[v];
    vps.push_back(vp);
    bool on_bu_path = bestUtilityPoses.count(vp) > 0;
    for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
    {
      ViewposePtr target = graph[ei->m_target];
      if (visualizedSet.count(ei->m_target) > 0) // already visualized
        continue;

      bool visualize = false;
      rviz_visual_tools::colors line_color = rviz_visual_tools::DEFAULT;
      if (on_bu_path && bestUtilityPoses.count(target) > 0) // is on best utility path
      {
        line_color = rviz_visual_tools::CYAN;
        verts_bu++;
        visualize = true;
      }
      else  if (vp->visited && target->visited) // has already been visited
      {
        line_color = rviz_visual_tools::BLACK;
        verts_vis++;
        visualize = true;
      }
      else if (target->pred != nullptr) // has been expanded from this node
      {
        line_color = rviz_visual_tools::DARK_GREY;
        verts_exp++;
        visualize = visualize_expanded;
      }
      else // not yet expanded
      {
        line_color = rviz_visual_tools::GREY;
        verts_ue++;
        visualize = visualize_unexpanded;
      }

      if (visualize)
        vt_searched_graph->publishLine(graph[ei->m_source]->pose.position, graph[ei->m_target]->pose.position, line_color);

      toVisualizeQueue.push(ei->m_target);
    }
  }
  ROS_INFO_STREAM("Visualization done; " << verts_bu << " bu, " << verts_vis << " vis, " << verts_exp << " exp, " << verts_ue << " ue");
  vt_searched_graph->trigger();
  publishViewposeVisualizations(vps);
}

void ViewposeGraphManager::initStartPose(const Vertex &v)
{
  current_start_vertex = v;
  current_start_vertex_number++;
  priorityQueue.push(v);
  expanded_vertices.insert(v);
}

void ViewposeGraphManager::markAsVisited(const Vertex &v)
{
  graph[v]->visited = true;
}

void ViewposeGraphManager::cleanupAfterMove()
{
  boost::unique_lock lock(graph_mtx);
  markAsVisited(current_start_vertex);
  priorityQueue.clear();
  for (const Vertex &v : expanded_vertices)
  {
    if (graph[v])
      graph[v]->clearPredecessor();
  }
  expanded_vertices.clear();
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
    if (ei->m_target == current_start_vertex || target->visited) // not a valid vertex to visit
    {
      continue;
    }
    if (target->pred) // already visited before
    {
      //TODO rewire
      //if (vp->accumulated_infogain > target->pred->accumulated_infogain) // set new predecessor
      continue;
    }
    target->addPredecessor(vp, t);
    if (!target->visited) // don't compute new cells for already visited targets
      computePoseObservedCells(mapping_manager, octomap_vpp::poseToOctomath(target->pose), target->freeCells, target->occCells, target->unkCells);

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

std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr> ViewposeGraphManager::getNextTrajectory()
{
  boost::shared_lock lock(graph_mtx);
  ViewposePtr vp = config.goal_select_type == Vmp_BEST_UTILITY ? highest_util_pose : highest_ig_pose;
  if (vp)
  {
    while(vp->pred && vp->pred->pred != nullptr)
    {
      vp = vp->pred;
    }
  }
  robot_trajectory::RobotTrajectoryPtr traj = getTrajectoryForState(vp->pred_edge, vp->pred);
  return {vertex_map[vp], traj};
}

std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> ViewposeGraphManager::getNextTrajectories(double cost_limit)
{
  boost::shared_lock lock(graph_mtx);
  ViewposePtr vp = config.goal_select_type == Vmp_BEST_UTILITY ? highest_util_pose : highest_ig_pose;
  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> next_trajectories;
  if (vp)
  {
    while(vp->pred)
    {
      if (vp->accumulated_cost < cost_limit || vp->pred->pred == nullptr) // always add at least one trajectory if available
      {
        next_trajectories.push_back({vertex_map[vp], getTrajectoryForState(vp->pred_edge, vp->pred), vp->pred_edge->cost});
      }
      vp = vp->pred;
    }
  }
  std::reverse(next_trajectories.begin(), next_trajectories.end());
  return next_trajectories;
}

bool ViewposeGraphManager::fixTrajectoryStartPoint(const robot_trajectory::RobotTrajectoryPtr &traj, const moveit::core::RobotStatePtr &state)
{
  moveit::core::RobotStatePtr cur_start = traj->getFirstWayPointPtr();
  traj->addPrefixWayPoint(state, 0);
  trajectory_processing::TimeOptimalTrajectoryGeneration trajectory_processor;
  bool success = trajectory_processor.computeTimeStamps(*traj, 1.0, 1.0);
  return success;
}

} // namespace view_motion_planner
