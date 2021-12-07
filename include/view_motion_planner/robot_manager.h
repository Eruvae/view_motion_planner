#pragma once

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class RobotManager
{
private:
  tf2_ros::Buffer &tfBuffer;
  MoveGroupInterface manipulator_group;
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  const robot_state::JointModelGroup* joint_model_group;
  robot_state::RobotStatePtr kinematic_state;

  const std::string pose_reference_frame;
  const std::string end_effector_link;
  std::vector<double> joint_start_values;
  bool start_values_set;

  bool planAndExecute(bool async, double *plan_length=nullptr, double *traj_duration=nullptr);

public:
  RobotManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &pose_reference_frame = "world",
                  const std::string& group_name = "manipulator", const std::string &ee_link_name = "camera_link");

  bool getCurrentTransform(geometry_msgs::TransformStamped &cur_tf);
  std::vector<double> getCurrentJointValues();
  bool reset();

  // Get robot states
  moveit::core::RobotStatePtr getPoseRobotState(const geometry_msgs::Pose &pose);
  moveit::core::RobotStatePtr getJointValueRobotState(const std::vector<double> &joint_values);

  // Get joint configurations
  std::vector<double> getPoseJointValues(const geometry_msgs::Pose &pose);

  // Direct move methods
  bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToPoseRelative(const geometry_msgs::Pose &relative_pose, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToState(const robot_state::RobotState &goal_state, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToState(const std::vector<double> &joint_values, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToStateRelative(const std::vector<double> &relative_joint_values, bool async=false, double *plan_length=nullptr, double *traj_duration=nullptr);
  bool moveToRandomTarget(bool async=false, const ros::Duration &timeout=ros::Duration(60));
};

} // namespace view_motion_planner
