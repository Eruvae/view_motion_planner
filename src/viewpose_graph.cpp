#include "view_motion_planner/viewpose_graph.h"

//#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/datastructures/NearestNeighborsLinear.h>
//#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

namespace view_motion_planner
{

ViewposeGraphManager::ViewposeGraphManager(const moveit::core::RobotModelConstPtr &kinematic_model, const robot_state::JointModelGroup* jmg)
  : kinematic_model(kinematic_model), jmg(jmg), neighbor_data(new ompl::NearestNeighborsGNAT<Vertex>())
{
  neighbor_data->setDistanceFunction(boost::bind(&ViewposeGraphManager::getVertexDistanceJoints, this, _1, _2));
}

double ViewposeGraphManager::getVertexDistancePose(Vertex a, Vertex b)
{
  const geometry_msgs::Point &pa = graph[a].pose.position;
  const geometry_msgs::Point &pb = graph[b].pose.position;
  double x = pb.x - pa.x;
  double y = pb.y - pa.y;
  double z = pb.z - pa.z;
  return x*x + y*y + z*z;
}

double ViewposeGraphManager::getVertexDistanceJoints(Vertex a, Vertex b)
{
  return graph[a].state->distance(*(graph[b].state), jmg);
}

void ViewposeGraphManager::connectNeighbors(const Vertex &v, size_t num_neighbors)
{
  std::vector<Vertex> neighbors;
  neighbor_data->nearestK(v, num_neighbors, neighbors);

  for (const Vertex &nv : neighbors)
  {
    // TODO check collision, generate trajectory
    Trajectory t;
    t.traj.reset(new robot_trajectory::RobotTrajectory(kinematic_model, jmg));
    t.traj->addSuffixWayPoint(graph[v].state, 0);
    t.traj->addSuffixWayPoint(graph[nv].state, 1);
    std::pair<Edge, bool> e = boost::add_edge(v, nv, t, graph);
  }
}

} // namespace view_motion_planner
