#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <ompl/datastructures/NearestNeighbors.h>
#include "view_motion_planner/robot_manager.h"

namespace view_motion_planner
{

struct Viewpose
{
  moveit::core::RobotStatePtr state;
  geometry_msgs::Pose pose;
};

struct Trajectory
{
  robot_trajectory::RobotTrajectoryPtr traj;
  double cost;
};

using ViewposeGraph = boost::adjacency_list<
boost::hash_setS, // OutEdgeListSelector
boost::vecS, // VertexListSelector
boost::undirectedS, // DirectedSelector
Viewpose, // VertexProperty
Trajectory, // EdgeProperty
boost::no_property, // GraphProperty
boost::listS>; // EdgeListSelector

using Vertex = boost::graph_traits<ViewposeGraph>::vertex_descriptor;
using Edge = boost::graph_traits<ViewposeGraph>::edge_descriptor;

class ViewposeGraphManager
{
private:
  ViewposeGraph graph;

  std::shared_ptr<RobotManager> robot_manager;

  std::unique_ptr<ompl::NearestNeighbors<Vertex>> neighbor_data;

  double getVertexDistancePose(Vertex a, Vertex b);
  double getVertexDistanceJoints(Vertex a, Vertex b);

public:
  ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager);

  const ViewposeGraph& getGraph() const
  {
    return graph;
  }

  void addViewpose(const Viewpose &vp)
  {
    Vertex v = boost::add_vertex(vp, graph);
    neighbor_data->add(v);
  }

  void connectNeighbors(const Vertex &v, size_t num_neighbors=5, double max_traj_length = 5.0, double traj_step = 0.1);

};

} // namespace view_motion_planner
