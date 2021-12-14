#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <ompl/datastructures/NearestNeighbors.h>

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
boost::hash_setS, // VertexListSelector
boost::directedS, // DirectedSelector
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

  moveit::core::RobotModelConstPtr kinematic_model;
  const robot_state::JointModelGroup* jmg;

  std::unique_ptr<ompl::NearestNeighbors<Vertex>> neighbor_data;

  double getVertexDistancePose(Vertex a, Vertex b);
  double getVertexDistanceJoints(Vertex a, Vertex b);

public:
  ViewposeGraphManager(const moveit::core::RobotModelConstPtr &kinematic_model, const robot_state::JointModelGroup* jmg);

  const ViewposeGraph& getGraph() const
  {
    return graph;
  }

  void addViewpose(const Viewpose &vp)
  {
    Vertex v = boost::add_vertex(vp, graph);
    neighbor_data->add(v);
  }

  void connectNeighbors(const Vertex &v, size_t num_neighbors=5);

};

} // namespace view_motion_planner
