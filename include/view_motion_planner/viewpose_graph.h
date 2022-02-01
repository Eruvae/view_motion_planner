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

class ViewposeGraphManager
{
private:
  ViewposeGraph graph;

  boost::shared_mutex graph_mtx;

  std::shared_ptr<RobotManager> robot_manager;

  std::unique_ptr<ompl::NearestNeighbors<Vertex>> neighbor_data;

  double getVertexDistancePose(Vertex a, Vertex b);
  double getVertexDistanceJoints(Vertex a, Vertex b);

public:
  ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager);

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
    return v;
  }

  void clear()
  {
    graph.clear();
    neighbor_data->clear();
  }

  void connectNeighbors(const Vertex &v, size_t num_neighbors=5, double max_traj_length = 5.0, double traj_step = 0.1);

};

class VertexUtilityComp
{
private:
  const ViewposeGraphManager &graph_manager;

public:
  VertexUtilityComp(const ViewposeGraphManager &graph_manager) : graph_manager(graph_manager) {}

  bool operator() (const Vertex &v1, const Vertex &v2) const
  {
    const ViewposePtr &vp1 = graph_manager.getGraph()[v1];
    const ViewposePtr &vp2 = graph_manager.getGraph()[v2];
    return vp1->accumulated_utility > vp2->accumulated_utility;
  }
};

class ViewposePathSearcher
{
private:
  ViewposeGraphManager &graph_manager;
  std::shared_ptr<OctreeManager> octree_manager;

  Vertex current_start_vertex;
  ViewposePtr highest_utility_pose;
  rviz_visual_tools::RvizVisualToolsPtr vt_searched_graph;

public:
  typedef boost::heap::fibonacci_heap<Vertex, boost::heap::compare<VertexUtilityComp>> ValueHeap;
  typedef std::unordered_map<Vertex, ValueHeap::handle_type> VertexHandleMap;

  ValueHeap priorityQueue;

  ViewposePathSearcher(ViewposeGraphManager &graph_manager, const std::shared_ptr<OctreeManager> &octree_manager,
                       const rviz_visual_tools::RvizVisualToolsPtr &vt_searched_graph) :
    graph_manager(graph_manager), octree_manager(octree_manager), vt_searched_graph(vt_searched_graph),
    priorityQueue(VertexUtilityComp(graph_manager))
  {

  }

  VertexHandleMap openVertices;
  std::unordered_set<Vertex> processedVertices;

  void visualizeGraph()
  {
    vt_searched_graph->deleteAllMarkers();
    graph_manager.getGraphMutex().lock_shared();
    std::queue<Vertex> toVisualizeQueue;
    std::unordered_set<ViewposePtr> bestUtilityPoses;
    std::unordered_set<Vertex> visualizedSet;
    toVisualizeQueue.push(current_start_vertex);
    for (ViewposePtr vp = highest_utility_pose; vp != nullptr; vp=vp->pred)
    {
      bestUtilityPoses.insert(vp);
    }
    while(!toVisualizeQueue.empty())
    {
      Vertex v = toVisualizeQueue.front();
      toVisualizeQueue.pop();
      visualizedSet.insert(v);
      ViewposePtr vp = graph_manager.getGraph()[v];
      for (auto [ei, ei_end] = boost::out_edges(v, graph_manager.getGraph()); ei != ei_end; ei++)
      {
        ViewposePtr target = graph_manager.getGraph()[ei->m_target];
        if (visualizedSet.find(ei->m_target) == visualizedSet.end()) // already visualized
          continue;

        rviz_visual_tools::colors line_color = rviz_visual_tools::DEFAULT;
        if (bestUtilityPoses.find(target) != bestUtilityPoses.end()) // is on best utility path
          line_color = rviz_visual_tools::CYAN;
        else if (target->pred != nullptr) // has been expanded
          line_color = rviz_visual_tools::DARK_GREY;
        else // not yet expanded
          line_color = rviz_visual_tools::GREY;

        vt_searched_graph->publishLine(graph_manager.getGraph()[ei->m_source]->pose.position, graph_manager.getGraph()[ei->m_target]->pose.position, line_color);
        toVisualizeQueue.push(ei->m_target);
      }
    }
    graph_manager.getGraphMutex().unlock_shared();
    vt_searched_graph->trigger();
  }

  void initStartPose(const Vertex &v)
  {
    current_start_vertex = v;
    ValueHeap::handle_type handle = priorityQueue.push(v);
    openVertices[v] = handle;
  }

  bool expand()
  {
    if (priorityQueue.empty())
      return false;

    Vertex v = priorityQueue.top();
    ViewposePtr vp = graph_manager.getGraph()[v];
    priorityQueue.pop();

    graph_manager.getGraphMutex().lock();
    moveit::core::RobotStatePtr cur_state = vp->state;
    for (auto [ei, ei_end] = boost::out_edges(v, graph_manager.getGraph()); ei != ei_end; ei++)
    {

      TrajectoryPtr t = graph_manager.getGraph()[*ei];
      const robot_trajectory::RobotTrajectoryPtr &traj = cur_state == t->traj->getFirstWayPointPtr() ? t->traj : t->bw_traj;

      ViewposePtr target = graph_manager.getGraph()[ei->m_target];
      if (target->pred || ei->m_target == current_start_vertex) // already visited before
      {
        // TODO: check if improved path
        continue;
      }
      target->addPredecessor(vp, t);
      octree_manager->computePoseObservedCells(octomap_vpp::poseToOctomath(target->pose), target->freeCells, target->occCells, target->unkCells);
      target->computeUtility();
      if (!highest_utility_pose || target->accumulated_utility > highest_utility_pose->accumulated_utility)
      {
        highest_utility_pose = target;
      }
      ValueHeap::handle_type handle = priorityQueue.push(ei->m_target);
      //robot_manager->executeTrajectory(traj);
    }
    graph_manager.getGraphMutex().unlock();
  }
};

} // namespace view_motion_planner
