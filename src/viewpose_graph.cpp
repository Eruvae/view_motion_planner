#include "view_motion_planner/viewpose_graph.h"

//#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/datastructures/NearestNeighborsLinear.h>
//#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

namespace view_motion_planner
{

ViewposeGraphManager::ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager)
  : robot_manager(robot_manager), neighbor_data(new ompl::NearestNeighborsGNAT<Vertex>())
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
  return graph[a].state->distance(*(graph[b].state), robot_manager->getJointModelGroup());
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

    moveit::core::RobotStatePtr from = graph[v].state;
    moveit::core::RobotStatePtr to = graph[nv].state;

    Trajectory t;
    t.cost = from->distance(*to, robot_manager->getJointModelGroup());
    if (t.cost > max_traj_length) // don't compute trajectory
      continue;

    t.traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
    t.bw_traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
    t.traj->addSuffixWayPoint(from, 0);
    t.bw_traj->addPrefixWayPoint(from, 1);
    bool found_traj = true;
    if (t.cost > traj_step) // interpolate points
    {
      size_t num_steps = static_cast<size_t>(t.cost / traj_step);
      double frac_step = 1.0 / num_steps;
      for (size_t i = 1; i < num_steps; i++) // don't check first and last state (assume valid)
      {
        moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*from));
        from->interpolate(*to, i * frac_step, *temp_state, robot_manager->getJointModelGroup());
        temp_state->update();

        bool state_valid = robot_manager->isValid(temp_state);
        if (!state_valid) // don't add edge
        {
          found_traj = false;
          break;
        }
        t.traj->addSuffixWayPoint(temp_state, frac_step);
        t.bw_traj->addPrefixWayPoint(temp_state, 1 - frac_step);
      }
    }
    if (found_traj)
    {
      t.traj->addSuffixWayPoint(to, 1);
      t.bw_traj->addPrefixWayPoint(to, 0);

      // TODO: time parameterization
      boost::add_edge(v, nv, t, graph);
    }
  }
}

} // namespace view_motion_planner
