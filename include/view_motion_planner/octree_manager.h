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
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/viewpose.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace view_motion_planner
{

class OctreeManager
{
private:
  ros::NodeHandle &nh;

  VmpConfig &config;

  std::shared_ptr<RobotManager> robot_manager;

  std::default_random_engine &random_engine;

  tf2_ros::Buffer &tfBuffer;
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> workspaceTree;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> samplingTree;
  octomap::point3d wsMin, wsMax;
  octomap::point3d stMin, stMax;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> observationRegions;
  std::shared_ptr<roi_viewpoint_planner::GtOctreeLoader> gtLoader;
  std::unique_ptr<roi_viewpoint_planner::Evaluator> evaluator;
  boost::mutex own_mtx;
  boost::mutex &tree_mtx;
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

  boost::mutex target_vector_mtx;
  std::vector<octomap::point3d> current_roi_targets;
  std::vector<octomap::point3d> current_expl_targets;
  std::vector<octomap::point3d> current_border_targets;

  const std::vector<octomath::Vector3> sphere_vecs;

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

  void registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg);

public:
  OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &wstree_file, const std::string &sampling_tree_file,
                const std::string &map_frame, const std::string &ws_frame, double tree_resolution, std::default_random_engine &random_engine,
                std::shared_ptr<RobotManager> robot_manager, VmpConfig &config, size_t num_sphere_vecs = 1000, bool initialize_evaluator=false);

  std::vector<ViewposePtr> sampleObservationPoses(double sensorRange=0.5);

  void waitForPointcloudWithRoi();

  void updateRoiTargets();
  bool getRandomRoiTarget(octomap::point3d &target);

  void updateExplTargets();
  bool getRandomExplTarget(octomap::point3d &target);
  bool getRandomBorderTarget(octomap::point3d &target);

  ViewposePtr sampleRandomViewPose(TargetType type, double minSensorRange, double maxSensorRange);

  //std::shared_ptr<octomap_vpp::WorkspaceOcTree> computeObservationRegions(double inflation_radius=0.2);

  std::shared_ptr<octomap_vpp::WorkspaceOcTree> getObservationRegions()
  {
    return observationRegions;
  }

  bool computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, octomap::KeyRay& ray)
  {
    tree_mtx.lock();
    bool ret = planningTree->computeRayKeys(origin, end, ray);
    tree_mtx.unlock();
    return ret;
  }

  bool computeRayCells(const octomap::point3d& origin, const octomap::point3d& end, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells)
  {
    tree_mtx.lock();
    octomap::KeyRay ray;
    bool ret = planningTree->computeRayKeys(origin, end, ray);
    if (!ret)
    {
      tree_mtx.unlock();
      return ret;
    }
    for (const octomap::OcTreeKey &key : ray)
    {
      const octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
      octomap_vpp::NodeState state = planningTree->getNodeState(node, octomap_vpp::NodeProperty::OCCUPANCY);
      if (state == octomap_vpp::NodeState::FREE_NONROI)
        freeCells.insert(key);
      else if (state == octomap_vpp::NodeState::UNKNOWN)
        unkCells.insert(key);
      else
      {
        occCells.insert(key);
        break;
      }
    }
    tree_mtx.unlock();
    return true;
  }

  void computePoseObservedCells(const octomap::pose6d &pose, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells)
  {
    octomap::point3d_collection endpoints = computeVpRaycastEndpoints(pose);
    for (const octomap::point3d &end : endpoints)
    {
      computeRayCells(pose.trans(), end, freeCells, occCells, unkCells);
    }
  }

  octomap::point3d transformToMapFrame(const octomap::point3d &p);
  geometry_msgs::Pose transformToMapFrame(const geometry_msgs::Pose &p);

  octomap::point3d transformToWorkspace(const octomap::point3d &p);
  geometry_msgs::Pose transformToWorkspace(const geometry_msgs::Pose &p);

  std::string saveOctomap(const std::string &name = "planningTree", bool name_is_prefix = true);

  int loadOctomap(const std::string &filename);

  void resetOctomap();

  void randomizePlants(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist);

  void publishMap();
  void publishObservationRegions();
  void publishObservationPoints(const octomap::KeySet &keys);
  void publishObservationPoints(const std::vector<ViewposePtr> &vps);

  bool startEvaluator();
  void setEvaluatorStartParams();
  bool saveEvaluatorData(double plan_length, double traj_duration);
  bool resetEvaluator();
};

} // namespace view_motion_planner
