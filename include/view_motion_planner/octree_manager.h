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
#include <rvp_evaluation/gt_octree_loader.h>
#include <rvp_evaluation/evaluator.h>
#include <rvp_evaluation/evaluator_external_clusters.h>
#include "view_motion_planner/vmp_utils.h"
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/viewpose.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_srvs/Empty.h>
#include "view_motion_planner/visualization_utils.h"

namespace view_motion_planner
{

class OctreeManager
{
private:
  ros::NodeHandle &nh;

  std::shared_ptr<RobotManager> robot_manager;

  std::default_random_engine &random_engine;

  tf2_ros::Buffer &tfBuffer;
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  std::shared_ptr<octomap_vpp::WorkspaceOcTree> observationRegions;
  std::shared_ptr<rvp_evaluation::GtOctreeLoader> gtLoader;
  std::unique_ptr<rvp_evaluation::Evaluator> evaluator;
  std::unique_ptr<rvp_evaluation::ExternalClusterEvaluator> external_cluster_evaluator;
  boost::mutex own_mtx;
  boost::mutex &tree_mtx;
  const std::string map_frame;
  const std::string ws_frame;
  ros::Publisher octomapPub;
  ros::Publisher observationRegionsPub;
  ros::Publisher observatonPointsPub;
  ros::Publisher targetPub;
  ros::Subscriber roiSub;
  size_t old_rois;
  octomap::KeySet encountered_keys;

  boost::mutex target_vector_mtx;
  std::vector<octomap::point3d> current_roi_targets;
  std::vector<octomap::point3d> current_expl_targets;
  std::vector<octomap::point3d> current_border_targets;

  const std::vector<octomath::Vector3> sphere_vecs;

  bool update_planning_tree;

  // Evaluator variables
  size_t eval_trial_num;
  std::ofstream eval_resultsFile;
  std::ofstream eval_resultsFileOld;
  std::ofstream eval_externalClusterFile;
  std::ofstream eval_fruitCellPercFile;
  std::ofstream eval_volumeAccuracyFile;
  std::ofstream eval_distanceFile;
  ros::Time eval_plannerStartTime;
  double eval_passedTime;
  double eval_accumulatedPlanDuration;
  double eval_accumulatedPlanLength;
  std::string eval_lastStep;

  ros::ServiceClient resetMoveitOctomapClient;
  ros::ServiceClient resetVoxbloxMapClient;
  std_srvs::Empty emptySrv;

  std::deque<ViewposePtr> past_viewposes_;

  void registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg);

public:
  OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer,
                const std::string &map_frame, const std::string &ws_frame, double tree_resolution, std::default_random_engine &random_engine,
                std::shared_ptr<RobotManager> robot_manager, size_t num_sphere_vecs = 1000,
                bool update_planning_tree=true, bool initialize_evaluator=false);

  double getEvalPassedTime()
  {
    return eval_passedTime;
  }

  double getEvalAccPlanDuration()
  {
    return eval_accumulatedPlanDuration;
  }

  double getEvalAccPlanLength()
  {
    return eval_accumulatedPlanLength;
  }

  std::vector<ViewposePtr> sampleObservationPoses(double sensorRange=0.5);

  void waitForPointcloudWithRoi();

  void updateRoiTargets();
  bool getRandomRoiTarget(octomap::point3d &target);

  void updateExplTargets();
  bool getRandomExplTarget(octomap::point3d &target);
  bool getRandomBorderTarget(octomap::point3d &target);

  void updateTargets();

  ViewposePtr sampleRandomViewPose(TargetType type);

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

  octomap::point3d sampleRandomWorkspacePoint()
  {
    std::uniform_real_distribution<float> x_dist(config.ws_min_x, config.ws_max_x);
    std::uniform_real_distribution<float> y_dist(config.ws_min_y, config.ws_max_y);
    std::uniform_real_distribution<float> z_dist(config.ws_min_z, config.ws_max_z);
    octomap::point3d target(x_dist(random_engine), y_dist(random_engine), z_dist(random_engine));
    return target;
  }

  template<typename PointT>
  PointT transformToMapFrame(const PointT &p)
  {
    if (map_frame == ws_frame)
      return p;

    geometry_msgs::TransformStamped trans;
    try
    {
      trans = tfBuffer.lookupTransform(map_frame, ws_frame, ros::Time(0));
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to ws frame in transformToMapFrame: " << e.what());
      return p;
    }

    PointT pt;
    tf2::doTransform(p, pt, trans);
    return pt;
  }

  template<typename PointT>
  PointT transformToWorkspace(const PointT &p)
  {
    if (map_frame == ws_frame)
      return p;

    geometry_msgs::TransformStamped trans;
    try
    {
      trans = tfBuffer.lookupTransform(ws_frame, map_frame, ros::Time(0));
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to ws frame in transformToWorkspace: " << e.what());
      return p;
    }

    PointT pt;
    tf2::doTransform(p, pt, trans);
    return pt;
  }

  bool isInWorkspace(const octomap::point3d &p)
  {
    return (p.x() >= config.ws_min_x && p.x() <= config.ws_max_x &&
            p.y() >= config.ws_min_y && p.y() <= config.ws_max_y &&
            p.z() >= config.ws_min_z && p.z() <= config.ws_max_z);
  }

  bool isInSamplingRegion(const octomap::point3d &p)
  {
    return (p.x() >= config.sr_min_x && p.x() <= config.sr_max_x &&
            p.y() >= config.sr_min_y && p.y() <= config.sr_max_y &&
            p.z() >= config.sr_min_z && p.z() <= config.sr_max_z);
  }

  std::string saveOctomap(const std::string &name = "planningTree", bool name_is_prefix = true);

  void moveOctomap(octomap_vpp::RoiOcTree* &tree, const geometry_msgs::Transform &offset);

  int loadOctomap(const std::string &filename, const geometry_msgs::Transform &offset=geometry_msgs::Transform());

  void resetOctomap();

  void randomizePlants(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist);

  void publishMap();
  void publishObservationRegions();
  void publishObservationPoints(const octomap::KeySet &keys);
  void publishObservationPoints(const std::vector<ViewposePtr> &vps);

//  bool startEvaluator();
//  void setEvaluatorStartParams();
//  bool saveEvaluatorData(double plan_length, double traj_duration);
//  bool resetEvaluator();

  inline void updatePastViewposesList(const ViewposePtr vp1)
  {
    if(past_viewposes_.size() < 1000)
    {
      past_viewposes_.push_back(vp1);
    }
    else
    {
      past_viewposes_.pop_front();
      past_viewposes_.push_back(vp1);
    }
  }

  inline void clearPastViewposesList()
  {
    past_viewposes_.clear();
  }
};

} // namespace view_motion_planner
