#pragma once

#include <ros/ros.h>
#include <random>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include "view_motion_planner/octree_manager.h"
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/viewpose_graph.h"

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class PauseCondition
{
  bool is_paused = false;
  boost::mutex pause_mutex;
  boost::condition_variable pause_change;

public:
  void wait()
  {
      boost::unique_lock<boost::mutex> lock(pause_mutex);
      while(is_paused)
      {
          pause_change.wait(lock);
      }
  }

  void setPaused(bool new_value)
  {
      {
          boost::unique_lock<boost::mutex> lock(pause_mutex);
          is_paused = new_value;
      }
      pause_change.notify_all();
  }
};

class ViewMotionPlanner
{
public:
  ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                    const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator=false);

  MoveGroupInterface::Plan getNextPlan();

  void poseVisualizeThread();

  void graphVisualizeThread();

  void graphBuilderThread();

  void computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells);

  Vertex initCameraPoseGraph();

  void pathSearcherThread();

  void pathExecuterThead();

  //void generateViewposeGraph();

  void initGraphBuilderThreads();

  void pauseGraphBuilderThreads();

  void resumeGraphBuilderThreads();

  void plannerLoop();

  //bool plannerLoopOnce(); // returns true if moved

  RobotManager* getRobotManager()
  {
    return robot_manager.get();
  }

private:
  std::default_random_engine random_engine;

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr vt_graph;
  rviz_visual_tools::RvizVisualToolsPtr vt_searched_graph;

  std::shared_ptr<RobotManager> robot_manager;
  moveit_visual_tools::MoveItVisualToolsPtr vt_robot_state;
  std::shared_ptr<OctreeManager> octree_manager;
  std::shared_ptr<ViewposeGraphManager> graph_manager;

  std::vector<ViewposePtr> observationPoses;
  boost::shared_mutex observationPoseMtx;

  bool graph_building_paused = false;
  boost::shared_mutex graph_building_pause_mutex;
  boost::condition_variable graph_building_pause_change;

  std::array<boost::thread, 4> graphBuilderThreads;
  PauseCondition graphBuilderPaused;
};

} // namespace view_motion_planner
