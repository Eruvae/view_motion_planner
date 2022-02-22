#pragma once

#include <ros/ros.h>
#include <random>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include "view_motion_planner/octree_manager.h"
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/viewpose_graph.h"
#include "view_motion_planner/VmpConfig.h"

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class PauseCondition
{
private:
  const size_t TOTAL_THREADS;
  bool is_paused = true;
  size_t waiting_threads = 0;
  boost::mutex pause_mutex;
  boost::condition_variable resume_threads;
  boost::condition_variable wait_for_pause;

public:
  PauseCondition(size_t number_of_threads) : TOTAL_THREADS(number_of_threads) {}

  void wait()
  {
      boost::unique_lock<boost::mutex> lock(pause_mutex);
      waiting_threads++;
      while(is_paused)
      {
        if (waiting_threads == TOTAL_THREADS)
        {
          wait_for_pause.notify_all();
        }
        resume_threads.wait(lock);
      }
      waiting_threads--;
  }

  void pause()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    is_paused = true;
  }

  void resume()
  {
    {
      boost::unique_lock<boost::mutex> lock(pause_mutex);
      is_paused = false;
    }
    resume_threads.notify_all();
  }

  void waitForPause()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    while (waiting_threads < TOTAL_THREADS)
    {
      wait_for_pause.wait(lock);
    }
  }

};

class ViewMotionPlanner
{
public:
  ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                    const std::string &map_frame, const std::string &ws_frame, double tree_resolution, size_t graph_builder_threads,
                    bool evaluation_mode=false, size_t eval_num_episodes=20, double eval_episode_duration=120.0);

  void poseVisualizeThread();

  void graphVisualizeThread();

  void graphBuilderThread();

  void computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells);

  Vertex initCameraPoseGraph();

  void pathSearcherThread(const ros::Time &end_time = ros::TIME_MAX);

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

  const size_t NUM_GRAPH_BUILDER_THREADS;

  bool evaluation_mode;
  size_t eval_num_episodes;
  double eval_episode_duration;

  VmpConfig config;
  boost::recursive_mutex config_mutex;
  dynamic_reconfigure::Server<VmpConfig> config_server;
  void reconfigureCallback(VmpConfig &config, uint32_t level);

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

  std::vector<boost::thread> graph_builder_threads;
  PauseCondition graph_builder_condition;
};

} // namespace view_motion_planner
