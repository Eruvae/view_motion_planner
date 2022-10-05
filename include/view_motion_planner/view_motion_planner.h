#pragma once

#include <ros/ros.h>
#include <random>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include "view_motion_planner/octree_manager.h"
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/viewpose_graph.h"
#include "roi_viewpoint_planner_msgs/VmpConfig.h"

#include "view_motion_planner/trolley_remote.h"

namespace view_motion_planner
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;

class PauseCondition
{
private:
  bool is_paused = true;
  bool do_shutdown = false;
  size_t started_threads = 0;
  size_t waiting_threads = 0;
  boost::mutex pause_mutex;
  boost::condition_variable resume_threads;
  boost::condition_variable wait_for_pause;
  boost::condition_variable wait_for_shutdown;

public:
  PauseCondition() {}

  void confirm_start()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    started_threads++;
  }

  void wait()
  {
      boost::unique_lock<boost::mutex> lock(pause_mutex);
      waiting_threads++;
      while(is_paused)
      {
        if (waiting_threads == started_threads)
        {
          wait_for_pause.notify_all();
        }
        resume_threads.wait(lock);
      }
      waiting_threads--;
      if (waiting_threads == 0)
      {
        wait_for_pause.notify_all();
      }
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

  void shutdown()
  {
    {
      boost::unique_lock<boost::mutex> lock(pause_mutex);
      is_paused = false;
      do_shutdown = true;
    }
    resume_threads.notify_all();
  }

  void confirm_shutdown()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    started_threads--;
    if (started_threads == 0)
    {
      wait_for_shutdown.notify_all();
    }
  }

  bool isShutdown()
  {
    return do_shutdown;
  }

  void waitForResume()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    while (started_threads > 0 && waiting_threads > 0)
    {
      wait_for_pause.wait(lock);
    }
  }

  void waitForShutdown()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    while (started_threads > 0)
    {
      wait_for_shutdown.wait(lock);
    }
  }

  void waitForPause()
  {
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    while (waiting_threads < started_threads)
    {
      wait_for_pause.wait(lock);
    }
  }

};

class ViewMotionPlanner
{
public:
  ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                    const std::string &map_frame, const std::string &ws_frame, const std::string &robot_description_param_name,
                    const std::string &group_name, const std::string &ee_link_name, double tree_resolution, size_t graph_builder_threads,
                    bool update_planning_tree=true, bool evaluation_mode=false, size_t eval_num_episodes=20,
                    EvalEpisodeEndParam ep=EvalEpisodeEndParam::TIME, double eval_episode_duration=120.0);

  ~ViewMotionPlanner();

  void poseVisualizeThread();

  void graphVisualizeThread();

  void graphBuilderThread();

  void computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells);

  std::optional<Vertex> initCameraPoseGraph();

  bool buildGraph();

  bool searchPath();

  bool executePath();

  void pathSearcherThread(EvalEpisodeEndParam ep, double duration);

  void pathSearcherThread(const ros::Time &end_time = ros::TIME_MAX);

  //void generateViewposeGraph();

  void initGraphBuilderThreads();

  void pauseGraphBuilderThreads();

  void resumeGraphBuilderThreads();

  void exploreNamedPoses();

  void plannerLoop();

  //bool plannerLoopOnce(); // returns true if moved

  RobotManager* getRobotManager()
  {
    return robot_manager.get();
  }

  OctreeManager* getOctreeManager()
  {
    return octree_manager.get();
  }

  ViewposeGraphManager* getGraphManager()
  {
    return graph_manager.get();
  }

  void updateConfig()
  {
    boost::recursive_mutex::scoped_lock lock(config_mutex);
    config_server.updateConfig(config);
  }

  VmpConfig& getConfig()
  {
    return config;
  }

private:
  std::default_random_engine random_engine;

  const size_t NUM_GRAPH_BUILDER_THREADS;

  bool update_planning_tree;

  bool evaluation_mode;
  size_t eval_num_episodes;
  EvalEpisodeEndParam ep;
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

  boost::thread pose_visualize_thread;
  boost::thread graph_visualize_thread;

  std::vector<boost::thread> graph_builder_threads;
  PauseCondition graph_builder_condition;
  PauseCondition pose_visualizer_condition;
  PauseCondition graph_visualizer_condition;

  trolley_remote::TrolleyRemote trolley_remote;
  int trolley_current_segment = 0;
};

} // namespace view_motion_planner
