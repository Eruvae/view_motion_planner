#include "view_motion_planner/view_motion_planner.h"

namespace view_motion_planner
{

MoveGroupInterface::Plan ViewMotionPlanner::getNextPlan()
{
  MoveGroupInterface::Plan plan;
  //TODO: compute plan
  return plan;
}

void ViewMotionPlanner::plannerLoop()
{
  for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }
}

bool ViewMotionPlanner::plannerLoopOnce()
{
  octree_manager->computeObservationRegions();
  octree_manager->publishMap();
  octree_manager->publishObservationRegions();
  return false;
}

} // namespace view_motion_planner
