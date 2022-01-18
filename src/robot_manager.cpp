#include "view_motion_planner/robot_manager.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <roi_viewpoint_planner/rvp_utils.h>

namespace view_motion_planner
{

RobotManager::RobotManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &pose_reference_frame, const std::string& group_name, const std::string &ee_link_name)
  : tfBuffer(tfBuffer),
    group_name(group_name),
    manipulator_group(group_name),
    rml(new robot_model_loader::RobotModelLoader("robot_description")),
    psm(new planning_scene_monitor::PlanningSceneMonitor(rml)),
    kinematic_model(rml->getModel()),
    jmg(kinematic_model->getJointModelGroup(group_name)),
    kinematic_state(new robot_state::RobotState(kinematic_model)),
    pose_reference_frame(pose_reference_frame),
    end_effector_link(ee_link_name)
{
  manipulator_group.setPoseReferenceFrame(pose_reference_frame);
  psm->startWorldGeometryMonitor();
  psm->startSceneMonitor();

  start_values_set = nh.getParam("/roi_viewpoint_planner/initial_joint_values", joint_start_values);

  if(!start_values_set)
  {
    ROS_WARN("No inital joint values set");
  }
}

bool RobotManager::planAndExecute(bool async, double *plan_length, double *traj_duration)
{
  MoveGroupInterface::Plan plan;
  ros::Time planStartTime = ros::Time::now();
  MoveItErrorCode res = manipulator_group.plan(plan);
  ROS_INFO_STREAM("Planning duration: " << (ros::Time::now() - planStartTime));
  if (res != MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Could not find plan");
    return false;
  }

  if (async)
    res = manipulator_group.asyncExecute(plan);
  else
    res = manipulator_group.execute(plan);

  if (plan_length)
    *plan_length = roi_viewpoint_planner::computeTrajectoryLength(plan);

  if (traj_duration)
    *traj_duration = roi_viewpoint_planner::getTrajectoryDuration(plan);

  /*if (res != MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Could not execute plan");
    return false;
  }*/
  return true;
}

bool RobotManager::getCurrentTransform(geometry_msgs::TransformStamped &cur_tf)
{
  try
  {
    cur_tf = tfBuffer.lookupTransform(pose_reference_frame, end_effector_link, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return false;
  }
  return true;
}

std::vector<double> RobotManager::getCurrentJointValues()
{
  return manipulator_group.getCurrentJointValues();
}

moveit::core::RobotStatePtr RobotManager::getCurrentState()
{
  return manipulator_group.getCurrentState();
}

geometry_msgs::Pose RobotManager::getCurrentPose()
{
  return manipulator_group.getCurrentPose(end_effector_link).pose;
}

bool RobotManager::reset()
{
  if (start_values_set)
    return moveToState(joint_start_values);

  return true;
}

moveit::core::RobotStatePtr RobotManager::getPoseRobotState(const geometry_msgs::Pose &pose)
{
  /* Set from IK options:
     \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be
      set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the \e tip  link in the chain needs to achieve
      @param tip The name of the link the pose is specified for
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions
     bool setFromIK(const JointModelGroup* group, const geometry_msgs::Pose& pose, double timeout = 0.0,
                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());
     bool setFromIK(const JointModelGroup* group, const geometry_msgs::Pose& pose, const std::string& tip,
                 double timeout = 0.0, const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());
  */

  moveit::core::RobotStatePtr state = manipulator_group.getCurrentState();
  if (!state->setFromIK(jmg, pose, end_effector_link))
  {
    return nullptr;
  }

  if (!isValid(state))
    return nullptr;

  return state;
}

moveit::core::RobotStatePtr RobotManager::getJointValueRobotState(const std::vector<double> &joint_values)
{
  moveit::core::RobotStatePtr state = manipulator_group.getCurrentState();
  state->setJointGroupPositions(jmg, joint_values);
  return state;
}

bool RobotManager::isValid(const moveit::core::RobotStatePtr &state)
{
  // Collision checking
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  planning_scene_monitor::LockedPlanningSceneRO scene(psm);
  scene->checkCollision(req, res, *state);
  return !res.collision;
}

std::vector<double> RobotManager::getPoseJointValues(const geometry_msgs::Pose &pose)
{
  std::vector<double> joint_values;
  manipulator_group_mtx.lock();
  if (!manipulator_group.setJointValueTarget(pose, end_effector_link))
  {
    manipulator_group_mtx.unlock();
    return joint_values;
  }
  manipulator_group.getJointValueTarget(joint_values);
  return joint_values;
}

bool RobotManager::moveToPose(const geometry_msgs::Pose &goal_pose, bool async, double *plan_length, double *traj_duration)
{
  ros::Time setTargetTime = ros::Time::now();
  if (!manipulator_group.setJointValueTarget(goal_pose, end_effector_link))
  {
    ROS_INFO_STREAM("Could not find IK for specified pose (Timeout: " << (ros::Time::now() - setTargetTime) << ")");
    return false;
  }
  ROS_INFO_STREAM("IK solve time: " << (ros::Time::now() - setTargetTime));

  return planAndExecute(async, plan_length, traj_duration);
}

bool RobotManager::moveToPoseRelative(const geometry_msgs::Pose &relative_pose, bool async, double *plan_length, double *traj_duration)
{
  geometry_msgs::TransformStamped cur_tf;
  bool success = getCurrentTransform(cur_tf);
  if (!success)
    return false;

  geometry_msgs::Pose goal_pose;
  tf2::doTransform(relative_pose, goal_pose, cur_tf);
  return moveToPose(goal_pose, async, plan_length, traj_duration);
}

bool RobotManager::moveToState(const robot_state::RobotState &goal_state, bool async, double *plan_length, double *traj_duration)
{
  manipulator_group.setJointValueTarget(goal_state);

  return planAndExecute(async, plan_length, traj_duration);
}

bool RobotManager::moveToState(const std::vector<double> &joint_values, bool async, double *plan_length, double *traj_duration)
{
  kinematic_state->setJointGroupPositions(jmg, joint_values);
  kinematic_state->enforceBounds();
  manipulator_group.setJointValueTarget(*kinematic_state);

  return planAndExecute(async, plan_length, traj_duration);
}

bool RobotManager::moveToStateRelative(const std::vector<double> &relative_joint_values, bool async, double *plan_length, double *traj_duration)
{
  std::vector<double> joint_values = manipulator_group.getCurrentJointValues();
  size_t joint_num = std::min(joint_values.size(), relative_joint_values.size());
  for (size_t i=0; i < joint_num; i++)
  {
    joint_values[i] += relative_joint_values[i];
  }
  return moveToState(joint_values, async, plan_length, traj_duration);
}

bool RobotManager::moveToRandomTarget(bool async, const ros::Duration &timeout)
{
  manipulator_group.setRandomTarget();
  bool success = false;
  ros::Time start_time = ros::Time::now();
  while(!success && ros::ok() && (ros::Time::now() - start_time) < timeout)
  {
    success = planAndExecute(async);
  }
  return success;
}

} // namespace view_motion_planner
