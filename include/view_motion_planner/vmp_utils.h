#pragma once

#include <octomap/octomap_types.h>
#include <tf2/utils.h>
#include <random>
#include <roi_viewpoint_planner_msgs/VmpConfig.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rvp_evaluation/compute_cubes.h> // !!!!!!!!!
#include <vector>
#include <visualization_msgs/Marker.h>
#include <octomap_vpp/RoiOcTree.h>
#include <Eigen/Dense>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <view_motion_planner/mapping_manager/mapping_node.h>

namespace view_motion_planner
{

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// TYPE DEFINITIONS ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// GLOBAL VARIABLES ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// TODO: lazy initialzed singleton?

// Global configuration; reconfiguration managed by ViewMotionPlanner
inline VmpConfig config;

inline std::random_device global_random_engine{};

// TODO: inline tf buffer?


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// FUNCTIONS USING GLOBAL VARIABLES ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

static inline bool isInWorkspace(const octomap::point3d &p)
{
  return (p.x() >= config.ws_min_x && p.x() <= config.ws_max_x &&
          p.y() >= config.ws_min_y && p.y() <= config.ws_max_y &&
          p.z() >= config.ws_min_z && p.z() <= config.ws_max_z);
}

static inline bool isInSamplingRegion(const octomap::point3d &p, octomap::point3d sr_min, octomap::point3d sr_max)
{
  return (p.x() >= sr_min.x() && p.x() <= sr_max.x() &&
          p.y() >= sr_min.y() && p.y() <= sr_max.y() &&
          p.z() >= sr_min.z() && p.z() <= sr_max.z());
}

static inline bool isInSamplingRegion(const octomap::point3d &p)
{
  return (p.x() >= config.sr_min_x && p.x() <= config.sr_max_x &&
          p.y() >= config.sr_min_y && p.y() <= config.sr_max_y &&
          p.z() >= config.sr_min_z && p.z() <= config.sr_max_z);
}


static inline bool getRandomTarget(const std::vector<octomap::point3d>& all_targets, octomap::point3d &out_target)
{
  if (all_targets.empty())
    return false;

  std::uniform_int_distribution<size_t> distribution(0, all_targets.size() - 1);
  out_target = all_targets[distribution(global_random_engine)];
  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// CONVERSION UTILITIES ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

static inline octomap::point3d toOctomath(const Eigen::Vector3d &point)
{
  return octomap::point3d(point(0), point(1), point(2));
}

static inline Eigen::Vector3d toEigen(const octomap::point3d &point)
{
  return Eigen::Vector3d(point.x(), point.y(), point.z());
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// VISUALIZATION UTILITIES /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// lambda initialization trick
// static const std_msgs::ColorRGBA COLOR_RED = []{std_msgs::ColorRGBA c; c.r = 1.f; c.g = 0.f; c.b = 0.f; c.a = 1.f; return c; } ();
// static const std_msgs::ColorRGBA COLOR_GREEN = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 1.f; c.b = 0.f; c.a = 1.f; return c; } ();
// static const std_msgs::ColorRGBA COLOR_BLUE = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 0.f; c.b = 1.f; c.a = 1.f; return c; } ();

// TODO: Not using. Markers are slow and difficult to manage on RViz.
// static inline visualization_msgs::Marker targetsToROSVisualizationMsg(const std::vector<octomap::point3d> &roi_targets, 
//                                                                       const std::vector<octomap::point3d> &expl_targets, 
//                                                                       const std::vector<octomap::point3d> &border_targets, 
//                                                                       const std::string& map_frame)
// {
//   const size_t NUM_P = roi_targets.size() + expl_targets.size() + border_targets.size();

//   visualization_msgs::Marker m;
//   m.header.frame_id = map_frame;
//   m.header.stamp = ros::Time();
//   m.ns = "targets";
//   m.id = 0;
//   m.type = visualization_msgs::Marker::POINTS;
//   m.action = visualization_msgs::Marker::ADD;
//   m.pose.orientation.w = 1.0;
//   m.scale.x = 0.005;
//   m.scale.y = 0.005;
//   m.color.a = 1.0;

//   if (NUM_P == 0)
//   {
//     ROS_WARN("No targets. Returning an empty visualization msg!");
//     return m;
//   }

//   m.points.reserve(NUM_P);
//   m.colors.reserve(NUM_P);

//   for (const octomap::point3d &p : roi_targets)
//   {
//     m.points.push_back(octomap::pointOctomapToMsg(p));
//     m.colors.push_back(COLOR_RED);
//   }
//   for (const octomap::point3d &p : expl_targets)
//   {
//     m.points.push_back(octomap::pointOctomapToMsg(p));
//     m.colors.push_back(COLOR_GREEN);
//   }
//   for (const octomap::point3d &p : border_targets)
//   {
//     m.points.push_back(octomap::pointOctomapToMsg(p));
//     m.colors.push_back(COLOR_BLUE);
//   }

//   return m;
// }

// PointT can be pcl::PointXYZ or pcl::PointXYZRGB. Set r,g,b values if using pcl::PointXYZRGB
template <typename PointT>
static inline sensor_msgs::PointCloud2Ptr targetsToPointCloud2Msg(const std::vector<octomap::point3d> &targets,
                                                                  const std::string& map_frame,
                                                                  uint8_t r = 0, uint8_t g = 0, uint8_t b = 0)
{
  sensor_msgs::PointCloud2Ptr cloud2(new sensor_msgs::PointCloud2());
  std::unique_ptr<pcl::PointCloud<PointT>> cloud(new pcl::PointCloud<PointT>());
  if (targets.size() <= 0) return cloud2;

  cloud->points.reserve(targets.size());
  for (int i = 0; i < targets.size(); ++i) {
    PointT pt;

    // An evil hack to check if PointT is PointXYZ or PointXYZRGB
    if (pcl::traits::has_color<PointT>())
    {
      pt = PointT(r,g,b);
    }

    pt.x = targets[i].x();
    pt.y = targets[i].y();
    pt.z = targets[i].z();
    cloud->points.push_back(pt);
  }

  pcl::toROSMsg(*cloud, *cloud2);
  cloud2->header.frame_id = map_frame;
  cloud2->header.stamp = ros::Time::now(); // TODO: ?
  return cloud2;
}







////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// OTHER UTILITIES /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// TODO!
// template<typename RandomEngine>
// static inline octomap::point3d sampleRandomWorkspacePoint(RandomEngine &engine)
// {
//   std::uniform_real_distribution<float> x_dist(config.ws_min_x, config.ws_max_x);
//   std::uniform_real_distribution<float> y_dist(config.ws_min_y, config.ws_max_y);
//   std::uniform_real_distribution<float> z_dist(config.ws_min_z, config.ws_max_z);
//   octomap::point3d target(x_dist(engine), y_dist(engine), z_dist(engine));
//   return target;
// }
static inline octomap::point3d sampleRandomWorkspacePoint()
{
  std::uniform_real_distribution<float> x_dist(config.ws_min_x, config.ws_max_x);
  std::uniform_real_distribution<float> y_dist(config.ws_min_y, config.ws_max_y);
  std::uniform_real_distribution<float> z_dist(config.ws_min_z, config.ws_max_z);
  octomap::point3d target(x_dist(global_random_engine), y_dist(global_random_engine), z_dist(global_random_engine));
  return target;
}


/*
 * Adapted from: https://stackoverflow.com/a/26127012
*/
static std::vector<octomath::Vector3> getFibonacciSphereVectors(size_t samples)
{
  std::vector<octomath::Vector3> vectors;
  vectors.reserve(samples);
  double phi = M_PI * (3.0 - std::sqrt(5.0));
  for (size_t i=0; i < samples; i++)
  {
    double y = 1.0 - (i / (double)(samples - 1)) * 2.0;
    double radius = std::sqrt(1.0 - y*y);
    double theta = phi * i;
    double x = std::cos(theta) * radius;
    double z = std::sin(theta) * radius;
    octomath::Vector3 vec(x, y, z);
    vectors.push_back(vec);
  }
  return vectors;
}

/**
 * Returns the given point on failure!
*/
template<typename PointT>
PointT transformToFrame(const PointT &p, const std::string &from, const std::string &to, const tf2_ros::Buffer &tfBuffer)
{
  if (from == to)
    return p;

  geometry_msgs::TransformStamped trans;
  try
  {
    trans = tfBuffer.lookupTransform(to, from, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform from " << from << " to " << to << " frame: " << e.what());
    return p;
  }

  PointT pt;
  tf2::doTransform(p, pt, trans);
  return pt;
}

/*
 * Algorithm from: Stark, Michael M. "Efficient construction of perpendicular vectors without branching." Journal of Graphics, GPU, and Game Tools 14.1 (2009): 55-62.
 * found in: https://blog.selfshadow.com/2011/10/17/perp-vectors/
*/
static inline tf2::Vector3 getPerpVectorStark(const tf2::Vector3 &u)
{
  tf2::Vector3 a = u.absolute();
  bool uyx = std::signbit(a.x() - a.y());
  bool uzx = std::signbit(a.x() - a.z());
  bool uzy = std::signbit(a.y() - a.z());

  bool xm = uyx & uzx;
  bool ym = (1^xm) & uzy;
  bool zm = 1^(xm & ym);

  tf2::Vector3 v = u.cross(tf2::Vector3(xm, ym, zm));
  return v;
}

template<typename RandomEngine>
octomap::point3d sampleRandomViewpoint(const octomap::point3d &target, double minDist, double maxDist, RandomEngine &engine)
{
  // Method 1
  /*std::normal_distribution<double> dir_distribution(0.0, 1.0);
  octomap::point3d p;
  do
  {
    for (size_t i = 0; i < 3; i++)
      p(i) = dir_distribution(engine);
  } while(p.norm_sq() < 0.0001);*/

  // Method 2
  std::uniform_real_distribution<double> z_dist(-1, 1);
  std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);
  double z = z_dist(engine);
  double theta = theta_dist(engine);
  double x = std::sin(theta)*std::sqrt(1 - z*z);
  double y = std::cos(theta)*std::sqrt(1 - z*z);
  octomap::point3d p(x, y, z);

  // Set length and offset
  p.normalize();
  std::uniform_real_distribution<double> dist_distribution(minDist, maxDist);
  p *= dist_distribution(engine);
  p += target;
  return p;
}

static inline tf2::Quaternion getQuatInDir(const octomath::Vector3 &dirVec)
{
  tf2::Vector3 x = tf2::Vector3(dirVec.x(), dirVec.y(), dirVec.z());
  tf2::Vector3 y = getPerpVectorStark(x);
  tf2::Vector3 z = x.cross(y);
  tf2::Matrix3x3 mat(x.x(), y.x(), z.x(),
                     x.y(), y.y(), z.y(),
                     x.z(), y.z(), z.z());
  tf2::Quaternion quat;
  mat.getRotation(quat);
  return quat;
}

// Check the code for hfov and vfov computation
static inline octomap::point3d_collection computeVpRaycastEndpoints(const octomap::pose6d &vp,
                                                             double hfov = 1.396263402,
                                                             double vfov = 1.047197551,
                                                             size_t x_steps = 9,
                                                             size_t y_steps = 7,
                                                             double maxRange = 1.0
                                                             )
{
  octomap::point3d_collection endpoints;
  //const double hfov = 80 * M_PI / 180.0; // 1,396263402
  //const double vfov = 60 * M_PI / 180.0; // 1,047197551
  //const size_t x_steps = 9;
  //const size_t y_steps = 7;
  //const double maxRange = 1.0;

  for (size_t i = 0; i < x_steps; i++)
  {
    double ha = -hfov/2 + (i / static_cast<double>(x_steps-1)) * hfov;
    for(size_t j = 0; j < y_steps; j++)
    {
      double va = -vfov/2 + (j / static_cast<double>(y_steps-1)) * vfov;
      octomap::point3d dir(1.0, tan(ha), tan(va));
      octomap::point3d end = dir * maxRange;
      endpoints.push_back(vp.transform(end));
    }
  }
  return endpoints;
}



static inline void countNodes(const std::vector<MappingNode> &nodes, int& unknown, int& free, int& non_roi, int& roi)
{
  for (const MappingNode& node: nodes)
  {
    if (node.state == MappingNodeState::ERROR)
    {
      ROS_WARN("countNodes: node state shouldn't be ERROR. Something is not ok.");
    }
    else if (node.state == MappingNodeState::UNKNOWN)
    {
      unknown++;
    }
    else if (node.state == MappingNodeState::FREE)
    {
      free++;
    }
    else if (node.state == MappingNodeState::NON_ROI)
    {
      non_roi++;
    }
    else if (node.state == MappingNodeState::ROI)
    {
      roi++;
    }
  }
}

static inline void moveOctomap(octomap_vpp::RoiOcTree* &tree, const geometry_msgs::Transform &offset)
{
  bool translate_tree = (offset.translation.x != 0 || offset.translation.y != 0 || offset.translation.z != 0);
  bool rotate_tree =  (offset.rotation.x != 0 || offset.rotation.y != 0 || offset.rotation.z != 0);

  if (!translate_tree && !rotate_tree)
    return;

  octomap_vpp::RoiOcTree *moved_tree(new octomap_vpp::RoiOcTree(tree->getResolution()));

  if (!rotate_tree) // no rotation needed, simply offset keys
  {
    const octomap::OcTreeKey ZERO_KEY = tree->coordToKey(0, 0, 0);
    const octomap::OcTreeKey OFFSET_KEY = tree->coordToKey(offset.translation.x, offset.translation.y, offset.translation.z);
    auto addOffset = [&ZERO_KEY, &OFFSET_KEY](const octomap::OcTreeKey &key)
    {
      return octomap::OcTreeKey(key[0] - ZERO_KEY[0] + OFFSET_KEY[0], key[1] - ZERO_KEY[1] + OFFSET_KEY[1], key[2] - ZERO_KEY[2] + OFFSET_KEY[2]);
    };
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; it++)
    {
      octomap::OcTreeKey insertKey = addOffset(it.getKey());
      octomap_vpp::RoiOcTreeNode *node = moved_tree->setNodeValue(insertKey, it->getLogOdds(), true);
      node->setRoiLogOdds(it->getRoiLogOdds());
    }
  }
  else // fully transform coordinate
  {
    octomap::pose6d transform = octomap_vpp::transformToOctomath(offset);
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; it++)
    {
      octomap::point3d insertPoint = transform.transform(it.getCoordinate());
      octomap_vpp::RoiOcTreeNode *node = moved_tree->setNodeValue(insertPoint, it->getLogOdds(), true);
      node->setRoiLogOdds(it->getRoiLogOdds());
    }
  }

  delete tree;
  tree = moved_tree;
}


template<typename T>
std::ostream& writeVector(std::ostream &os, double passed_time, const std::vector<T> &vec, const std::string separator = ",")
{
  os << passed_time << separator;
  for (size_t i = 0; i < vec.size(); i++)
  {
    os << vec[i];
    if (i < vec.size() - 1)
      os << separator;
  }
  return os;
}

static inline std::string generateFilename(const std::string &name, bool name_is_prefix)
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
  return fName.str();
}

} // namespace view_motion_planner
