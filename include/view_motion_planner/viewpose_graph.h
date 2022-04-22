#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <ompl/datastructures/NearestNeighbors.h>
#include "view_motion_planner/viewpose.h"
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/octree_manager.h"
#include "roi_viewpoint_planner_msgs/VmpConfig.h"

namespace view_motion_planner
{

using ViewposeGraph = boost::adjacency_list<
boost::hash_setS, // OutEdgeListSelector
boost::vecS, // VertexListSelector
boost::undirectedS, // DirectedSelector
ViewposePtr, // VertexProperty
TrajectoryPtr, // EdgeProperty
boost::no_property, // GraphProperty
boost::listS>; // EdgeListSelector

using Vertex = boost::graph_traits<ViewposeGraph>::vertex_descriptor;
using Edge = boost::graph_traits<ViewposeGraph>::edge_descriptor;

class ViewposeGraphManager;

class VertexUtilityComp
{
private:
  ViewposeGraphManager *graph_manager;

public:
  VertexUtilityComp(ViewposeGraphManager *graph_manager) : graph_manager(graph_manager) {}

  bool operator() (const Vertex &v1, const Vertex &v2) const;
};

class ViewposeGraphManager
{
private:
  ViewposeGraph graph;

  boost::shared_mutex graph_mtx;

  const VmpConfig &config;

  std::unordered_map<ViewposePtr, Vertex> vertex_map;

  std::shared_ptr<RobotManager> robot_manager;
  std::shared_ptr<OctreeManager> octree_manager;

  std::unique_ptr<ompl::NearestNeighbors<Vertex>> neighbor_data;

  Vertex current_start_vertex;
  ViewposePtr highest_ig_pose = nullptr;
  ViewposePtr highest_util_pose = nullptr;
  rviz_visual_tools::RvizVisualToolsPtr vt_searched_graph;

  typedef boost::heap::fibonacci_heap<Vertex, boost::heap::compare<VertexUtilityComp>> ValueHeap;
  typedef std::unordered_map<Vertex, ValueHeap::handle_type> VertexHandleMap;
  std::unordered_set<Vertex> expanded_vertices;
  size_t current_start_vertex_number;

  ValueHeap priorityQueue;

  double getVertexDistancePose(Vertex a, Vertex b);
  double getVertexDistanceJoints(Vertex a, Vertex b);

public:
  ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager,
                       const std::shared_ptr<OctreeManager> &octree_manager,
                       const rviz_visual_tools::RvizVisualToolsPtr &vt_searched_graph,
                       const VmpConfig &config);

  boost::shared_mutex &getGraphMutex()
  {
    return graph_mtx;
  }

  ViewposeGraph& getGraph()
  {
    return graph;
  }

  const ViewposeGraph& getGraph() const
  {
    return graph;
  }

  Vertex addViewpose(const ViewposePtr &vp)
  {
    Vertex v = boost::add_vertex(vp, graph);
    neighbor_data->add(v);
    vertex_map[vp] = v;
    return v;
  }

  void clear()
  {
    boost::unique_lock lock(graph_mtx);
    graph.clear();
    neighbor_data->clear();
    highest_ig_pose = nullptr;
    highest_util_pose = nullptr;
    vertex_map.clear();
    expanded_vertices.clear();
    current_start_vertex_number = 0;
    priorityQueue.clear();
  }

  void connectNeighbors(const Vertex &v, size_t num_neighbors, double max_traj_length, double traj_step = 0.1);

  void visualizeGraph(bool visualize_expanded, bool visualize_unexpanded);

  void initStartPose(const Vertex &v);

  void markAsVisited(const Vertex &v);

  void cleanupAfterMove();

  bool expand();

  Vertex getCurrentStartVertex()
  {
    return current_start_vertex;
  }

  std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr> getNextTrajectory();

  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> getNextTrajectories(double cost_limit);

};

} // namespace view_motion_planner
