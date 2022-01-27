#pragma once

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class RobotManager
{
private:
  tf2_ros::Buffer &tfBuffer;
  const std::string group_name;
  MoveGroupInterface manipulator_group;
  robot_model_loader::RobotModelLoaderPtr rml;
  planning_scene_monitor::PlanningSceneMonitorPtr psm;
  robot_model::RobotModelConstPtr kinematic_model;
  const robot_state::JointModelGroup* jmg;
  robot_state::RobotStatePtr kinematic_state;

  boost::mutex manipulator_group_mtx;

  const std::string pose_reference_frame;
  const std::string end_effector_link;
  std::vector<double> joint_start_values;
  bool start_values_set;

  bool planAndExecute(bool async, double *plan_length=nullptr, double *traj_duration=nullptr);

public:
  RobotManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &pose_reference_frame = "world",
                  const std::string& group_name = "manipulator", const std::string &ee_link_name = "camera_link");

  moveit::core::RobotModelConstPtr getRobotModel()
  {
    return kinematic_model;
  }

  const robot_state::JointModelGroup* getJointModelGroup()
  {
    return jmg;
  }

  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    return psm;
  }

  bool getCurrentTransform(geometry_msgs::TransformStamped &cur_tf);
  std::vector<double> getCurrentJointValues();
  moveit::core::RobotStatePtr getCurrentState();
  geometry_msgs::Pose getCurrentPose();
  bool reset();

  // Get robot states / poses
  moveit::core::RobotStatePtr getPoseRobotState(const geometry_msgs::Pose &pose);
  moveit::core::RobotStatePtr getJointValueRobotState(const std::vector<double> &joint_values);

  // TODO: not working
  octomap::pose6d getRobotStatePose(const moveit::core::RobotStatePtr &state);

  // Collision check
  bool isValid(const moveit::core::RobotStatePtr &state);

  // Get joint configurations
  std::vector<double> getPoseJointValues(const geometry_msgs::Pose &pose);

  // Direct move methods
  bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToPoseRelative(const geometry_msgs::Pose &relative_pose, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToState(const robot_state::RobotState &goal_state, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToState(const std::vector<double> &joint_values, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToStateRelative(const std::vector<double> &relative_joint_values, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToRandomTarget(bool async=false, const ros::Duration &timeout=ros::Duration(60));

  bool executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &traj);
};

} // namespace view_motion_planner
