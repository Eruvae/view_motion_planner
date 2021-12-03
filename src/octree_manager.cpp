#include "view_motion_planner/octree_manager.h"
#include <octomap_msgs/conversions.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/counting_range.hpp>
#include <execution>
#include <roi_viewpoint_planner/planner_interfaces/provided_tree_interface.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace view_motion_planner
{

OctreeManager::OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &map_frame, double tree_resolution, bool initialize_evaluator) :
  tfBuffer(tfBuffer), planningTree(new octomap_vpp::RoiOcTree(tree_resolution)), observationRegions(new octomap_vpp::WorkspaceOcTree(tree_resolution)),
  gtLoader(new roi_viewpoint_planner::GtOctreeLoader(tree_resolution)), evaluator(nullptr), map_frame(map_frame), old_rois(0), tree_mtx(own_mtx)
{
  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
  observationRegionsPub = nh.advertise<octomap_msgs::Octomap>("observation_regions", 1);
  observatonPointsPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("observation_points", 1);
  roiSub = nh.subscribe("/detect_roi/results", 1, &OctreeManager::registerPointcloudWithRoi, this);

  if (initialize_evaluator)
  {
    ros::NodeHandle nh_eval("evaluator");
    std::shared_ptr<roi_viewpoint_planner::ProvidedTreeInterface> interface(new roi_viewpoint_planner::ProvidedTreeInterface(planningTree, tree_mtx));
    evaluator.reset(new roi_viewpoint_planner::Evaluator(interface, nh, nh_eval, true, false, gtLoader));
    eval_trial_num = 0;
    evaluator->saveGtAsColoredCloud();
  }
}

OctreeManager::OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer, const std::string &map_frame,
              const std::shared_ptr<octomap_vpp::RoiOcTree> &providedTree, boost::mutex &tree_mtx, bool initialize_evaluator) :
  tfBuffer(tfBuffer), planningTree(providedTree), observationRegions(new octomap_vpp::WorkspaceOcTree(providedTree->getResolution())),
  gtLoader(new roi_viewpoint_planner::GtOctreeLoader(providedTree->getResolution())), evaluator(nullptr), map_frame(map_frame), old_rois(0), tree_mtx(tree_mtx)
{
  if (initialize_evaluator)
  {
    ros::NodeHandle nh_eval("evaluator");
    std::shared_ptr<roi_viewpoint_planner::ProvidedTreeInterface> interface(new roi_viewpoint_planner::ProvidedTreeInterface(planningTree, tree_mtx));
    evaluator.reset(new roi_viewpoint_planner::Evaluator(interface, nh, nh_eval, true, false, gtLoader));
    eval_trial_num = 0;
    evaluator->saveGtAsColoredCloud();
  }
}

void OctreeManager::registerPointcloudWithRoi(const ros::MessageEvent<pointcloud_roi_msgs::PointcloudWithRoi const> &event)
{
  const pointcloud_roi_msgs::PointcloudWithRoi &roi = *event.getMessage();

  geometry_msgs::TransformStamped pcFrameTf;
  bool cloud_in_map_frame = (roi.cloud.header.frame_id == map_frame);
  if (cloud_in_map_frame)
  {
    //ROS_INFO_STREAM("Incoming cloud already in target frame");
    //pcFrameTf.header = roi.cloud.header;
    pcFrameTf.transform = roi.transform;
  }
  else
  {
    //ROS_INFO_STREAM("Convert incoming cloud (" << roi.cloud.header.frame_id << ") to map frame (" << map_frame << "), assuming no transform in incoming data");
    try
    {
      pcFrameTf = tfBuffer.lookupTransform(map_frame, roi.cloud.header.frame_id, roi.cloud.header.stamp);
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to map frame in registerRoiOCL: " << e.what());
      return;
    }
  }

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);

  octomap::Pointcloud inlierCloud, outlierCloud, fullCloud;
  if (cloud_in_map_frame)
    octomap_vpp::pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, inlierCloud, outlierCloud, fullCloud);
  else
    octomap_vpp::pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, pcFrameTf.transform, inlierCloud, outlierCloud, fullCloud);

  tree_mtx.lock();
  planningTree->insertPointCloud(fullCloud, scan_orig);
  planningTree->insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();
}

octomap::KeySet OctreeManager::sampleObservationPoints(double sensorRange)
{
  octomap::KeySet observationPoints;

  tree_mtx.lock();
  octomap::KeySet roiKeys = planningTree->getRoiKeys();
  for (octomap::OcTreeKey roiKey : roiKeys)
  {
    octomap::point3d origin = planningTree->keyToCoord(roiKey);
    for (const octomap::point3d &direction : sphere_vecs)
    {
      octomap::point3d end;
      // castRay(const point3d& origin, const point3d& direction, point3d& end, bool ignoreUnknownCells=false, double maxRange=-1.0) const;
      bool intersects = planningTree->castRay(origin, direction, end, true, sensorRange);
      if (!intersects)
        observationPoints.insert(planningTree->coordToKey(end));
    }
  }
  tree_mtx.unlock();

  return observationPoints;
}

std::shared_ptr<octomap_vpp::WorkspaceOcTree> OctreeManager::computeObservationRegions(double inflation_radius)
{
  struct KeyWithPrio
  {
    KeyWithPrio(const octomap::OcTreeKey &key, float prio) : key(key), prio(prio) {}

    octomap::OcTreeKey key;
    float prio;

    bool operator< (const KeyWithPrio &rhs) const
    {
      return prio < rhs.prio;
    }
  };

  typedef boost::heap::fibonacci_heap<KeyWithPrio> ValueHeap;
  typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ValueHeap::handle_type, octomap::OcTreeKey::KeyHash> KeyHandleMap;

  ValueHeap nodeVals;
  KeyHandleMap openKeys;
  octomap::KeySet processedKeys;

  octomap::KeySet observationPoints = sampleObservationPoints();
  publishObservationPoints(observationPoints);

  for (const octomap::OcTreeKey &key : observationPoints)
  {
    ValueHeap::handle_type handle = nodeVals.push(KeyWithPrio(key, 1.0));
    openKeys[key] = handle;
  }

  float stepReduction = 1.0 / inflation_radius * observationRegions->getResolution();

  ROS_INFO("Start inflation");

  ros::Time startTime(ros::Time::now());
  ros::Time lastPrintTime(startTime);

  while (!nodeVals.empty())
  {
    const KeyWithPrio &maxEl = nodeVals.top();
    octomap::OcTreeKey curKey = maxEl.key;
    float curVal = maxEl.prio;
    nodeVals.pop();
    openKeys.erase(curKey);
    observationRegions->updateNode(curKey, curVal, false);
    processedKeys.insert(curKey);

    ros::Time curTime(ros::Time::now());
    if (curTime - lastPrintTime > ros::Duration(1)) // print every second
    {
      ROS_INFO_STREAM("Estimated progress: " << ((1.0 - curVal) * 100.0) << "%");
      lastPrintTime = curTime;
      if (!ros::ok())
        break;
    }

    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        for (int k = -1; k <= 1; k++)
        {
          int coordDiff = std::abs(i) + std::abs(j) + std::abs(k);
          float newNeighbourVal = 0;
          if (coordDiff == 0) continue; // only process neighbours, not the node itself
          else if (coordDiff == 1) newNeighbourVal = curVal - stepReduction;
          else if (coordDiff == 2) newNeighbourVal = curVal - stepReduction * sqrt(2);
          else /* coordDiff == 3*/ newNeighbourVal = curVal - stepReduction * sqrt(3);

          if (newNeighbourVal <= 0) continue; // new value is out of influence radius

          octomap::OcTreeKey neighbourKey(curKey[0] + i, curKey[1] + j, curKey[2] + k);
          if (processedKeys.find(neighbourKey) != processedKeys.end()) continue; // already processed keys can be ignored

          auto it = openKeys.find(neighbourKey);
          if (it != openKeys.end()) // key already known, check if update needed
          {
            ValueHeap::handle_type handle = it->second;
            if (newNeighbourVal > (*handle).prio) // only update if new value would be higher
            {
              (*handle).prio = newNeighbourVal;
              nodeVals.increase(handle);
            }
          }
          else // otherwise, enter new key to map
          {
            ValueHeap::handle_type handle = nodeVals.push(KeyWithPrio(neighbourKey, newNeighbourVal));
            openKeys[neighbourKey] = handle;
          }
        }
      }
    }
  }
  observationRegions->updateInnerVals();
  ros::Time endTime(ros::Time::now());
  ROS_INFO_STREAM("Inflation done, took " << (endTime - startTime).toSec() << "s");
  return observationRegions;
}

std::string OctreeManager::saveOctomap(const std::string &name, bool name_is_prefix)
{
  std::stringstream fName;
  fName << name;
  if (name_is_prefix)
  {
    const boost::posix_time::ptime curDateTime = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const timeFacet = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    fName.imbue(std::locale(fName.getloc(), timeFacet));
    fName << "_" << curDateTime;
  }
  fName << ".ot";
  tree_mtx.lock();
  bool result = planningTree->write(fName.str());
  tree_mtx.unlock();
  return result ? fName.str() : "";
}

int OctreeManager::loadOctomap(const std::string &filename)
{
  octomap_vpp::RoiOcTree *map = NULL;
  octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(filename);
  if (!tree)
    return -1;

  map = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
  if(!map)
  {
    delete tree;
    return -2;
  }
  tree_mtx.lock();
  planningTree.reset(map);
  planningTree->computeRoiKeys();
  tree_mtx.unlock();
  publishMap();
  return 0;
}

void OctreeManager::resetOctomap()
{
  tree_mtx.lock();
  planningTree->clear();
  planningTree->clearRoiKeys();
  old_rois = 0;
  tree_mtx.unlock();
  encountered_keys.clear();
  publishMap();
}

void OctreeManager::randomizePlants(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist)
{
  if (evaluator)
    evaluator->randomizePlantPositions(octomap_vpp::pointToOctomath(min), octomap_vpp::pointToOctomath(max), min_dist);
  else
    gtLoader->randomizePlantPositions(octomap_vpp::pointToOctomath(min), octomap_vpp::pointToOctomath(max), min_dist);
}

void OctreeManager::publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(*planningTree, map_msg);
  tree_mtx.unlock();
  if (msg_generated)
  {
    octomapPub.publish(map_msg);
  }
}

void OctreeManager::publishObservationRegions()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  bool msg_generated = octomap_msgs::fullMapToMsg(*observationRegions, map_msg);
  if (msg_generated)
  {
    observationRegionsPub.publish(map_msg);
  }
}

void OctreeManager::publishObservationPoints(const octomap::KeySet &keys)
{
  //octomap::point3d (octomap_vpp::WorkspaceOcTree::*keyToCoord)(const octomap::OcTreeKey &key) const = &octomap_vpp::WorkspaceOcTree::keyToCoord;
  auto keyToCoord = [this](const octomap::OcTreeKey &key) {return observationRegions->keyToCoord(key);};
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc = octomap_vpp::octomapKeysetToPcl<pcl::PointXYZ>(keys, keyToCoord);
  pc->header.frame_id = map_frame;
  pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp);
  observatonPointsPub.publish(pc);
}

bool OctreeManager::startEvaluator()
{
  eval_trial_num = 0;
  setEvaluatorStartParams();
  return true;
}

void OctreeManager::setEvaluatorStartParams()
{
  std::string file_index_str = std::to_string(eval_trial_num);
  eval_resultsFile = std::ofstream("planner_results_" + file_index_str + ".csv");
  eval_resultsFileOld = std::ofstream("planner_results_old" + file_index_str + ".csv");
  eval_fruitCellPercFile = std::ofstream("results_fruit_cells_" + file_index_str + ".csv");
  eval_volumeAccuracyFile = std::ofstream("results_volume_accuracy_" + file_index_str + ".csv");
  eval_distanceFile = std::ofstream("results_distances_" + file_index_str + ".csv");
  eval_resultsFile << "Time (s),Plan duration (s),Plan Length,";
  evaluator->writeHeader(eval_resultsFile) << ",Step" << std::endl;
  eval_resultsFileOld << "Time (s),Plan duration (s),Plan Length,";
  evaluator->writeHeaderOld(eval_resultsFileOld) << ",Step" << std::endl;
  eval_plannerStartTime = ros::Time::now();
  eval_accumulatedPlanDuration = 0;
  eval_accumulatedPlanLength = 0;
}

template<typename T>
std::ostream& writeVector(std::ostream &os, double passed_time, const std::vector<T> &vec)
{
  os << passed_time << ",";
  for (size_t i = 0; i < vec.size(); i++)
  {
    os << vec[i];
    if (i < vec.size() - 1)
      os << ",";
  }
  return os;
}

bool OctreeManager::saveEvaluatorData(double plan_length, double traj_duration)
{
  ros::Time currentTime = ros::Time::now();

  double passed_time = (currentTime - eval_plannerStartTime).toSec();

  eval_accumulatedPlanDuration += traj_duration;
  eval_accumulatedPlanLength += plan_length;

  roi_viewpoint_planner::EvaluationParameters res = evaluator->processDetectedRois(true, eval_trial_num, static_cast<size_t>(passed_time));
  roi_viewpoint_planner::EvaluationParametersOld resOld = evaluator->processDetectedRoisOld();

  eval_resultsFile << passed_time << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  evaluator->writeParams(eval_resultsFile, res) << "," << eval_lastStep << std::endl;

  eval_resultsFileOld << passed_time << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  evaluator->writeParamsOld(eval_resultsFileOld, resOld) << "," << eval_lastStep << std::endl;

  writeVector(eval_fruitCellPercFile, passed_time, res.fruit_cell_percentages) << std::endl;
  writeVector(eval_volumeAccuracyFile, passed_time, res.volume_accuracies) << std::endl;
  writeVector(eval_distanceFile, passed_time, res.distances) << std::endl;

  return true;
}

bool OctreeManager::resetEvaluator()
{
  eval_resultsFile.close();
  eval_resultsFileOld.close();
  eval_fruitCellPercFile.close();
  eval_volumeAccuracyFile.close();
  eval_distanceFile.close();
  eval_trial_num++;
  setEvaluatorStartParams();
  return true;
}

} // namespace view_motion_planner
