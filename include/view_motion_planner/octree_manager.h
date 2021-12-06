#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/message_event.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap_vpp/WorkspaceOcTree.h>
#include <octomap_msgs/Octomap.h>
#include "octomap_vpp/roioctree_utils.h"
#include <roi_viewpoint_planner/gt_octree_loader.h>
#include <roi_viewpoint_planner/evaluator.h>
#include "view_motion_planner/vmp_utils.h"

namespace view_motion_planner
{

class OctreeManager
{
private:
  tf2_ros::Buffer &tfBuffer;
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> workspaceTree;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> samplingTree;
  octomap::point3d wsMin, wsMax;
  octomap::point3d stMin, stMax;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> observationRegions;
  std::shared_ptr<roi_viewpoint_planner::GtOctreeLoader> gtLoader;
  std::unique_ptr<roi_viewpoint_planner::Evaluator> evaluator;
  boost::shared_mutex own_mtx;
  boost::shared_mutex &tree_mtx;
  const std::string map_frame;
  const std::string ws_frame;
  ros::Publisher octomapPub;
  ros::Publisher workspaceTreePub;
  ros::Publisher samplingTreePub;
  ros::Publisher observationRegionsPub;
  ros::Publisher observatonPointsPub;
  ros::Subscriber roiSub;
  size_t old_rois;
  octomap::KeySet encountered_keys;

  const std::vector<octomath::Vector3> sphere_vecs = getFibonacciSphereVectors(1000);

  // Evaluator variables
  size_t eval_trial_num;
  std::ofstream eval_resultsFile;
  std::ofstream eval_resultsFileOld;
  std::ofstream eval_fruitCellPercFile;
  std::ofstream eval_volumeAccuracyFile;
  std::ofstream eval_distanceFile;
  ros::Time eval_plannerStartTime;
  double eval_accumulatedPlanDuration;
  double eval_accumulatedPlanLength;
  std::string eval_lastStep;

  void registerPointcloudWithRoi(const ros::MessageEvent<pointcloud_roi_msgs::PointcloudWithRoi const> &event);

  octomap::KeySet sampleObservationPoints(double sensorRange=0.5);

public:
  // Constructor to store own tree, subscribe to pointcloud roi
  OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                const std::string &map_frame, const std::string &ws_frame, double tree_resolution, bool initialize_evaluator=false);

  // Constructor to pass existing tree + mutex, e.g. from viewpoint planner
  OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &map_frame,
                const std::shared_ptr<octomap_vpp::RoiOcTree> &providedTree, boost::shared_mutex &tree_mtx, bool initialize_evaluator=false);

  std::shared_ptr<octomap_vpp::WorkspaceOcTree> computeObservationRegions(double inflation_radius=0.2);

  std::shared_ptr<octomap_vpp::WorkspaceOcTree> getObservationRegions()
  {
    return observationRegions;
  }

  std::string saveOctomap(const std::string &name = "planningTree", bool name_is_prefix = true);

  int loadOctomap(const std::string &filename);

  void resetOctomap();

  void randomizePlants(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist);

  void publishMap();
  void publishObservationRegions();
  void publishObservationPoints(const octomap::KeySet &keys);

  bool startEvaluator();
  void setEvaluatorStartParams();
  bool saveEvaluatorData(double plan_length, double traj_duration);
  bool resetEvaluator();
};

} // namespace view_motion_planner
