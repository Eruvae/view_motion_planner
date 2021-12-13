#pragma once

#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <ompl/datastructures/NearestNeighbors.h>
//#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/datastructures/NearestNeighborsLinear.h>
//#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

namespace view_motion_planner
{

struct Viewpose
{
  moveit::core::RobotStatePtr state;
  geometry_msgs::Pose pose;
};

struct Trajectory
{
  double cost;
};

using ViewposeGraph = boost::adjacency_list<
boost::vecS, // OutEdgeListSelector
boost::vecS, // VertexListSelector
boost::directedS, // DirectedSelector
Viewpose, // VertexProperty
Trajectory, // EdgeProperty
boost::no_property, // GraphProperty
boost::listS>; // EdgeListSelector

class ViewposeGraphManager
{
private:
  ViewposeGraph graph;

public:
  void addViewpose(const Viewpose &vp)
  {
    auto t = boost::add_vertex(vp, graph);
  }
};

} // namespace view_motion_planner
