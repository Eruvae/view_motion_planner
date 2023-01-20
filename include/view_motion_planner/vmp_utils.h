#pragma once

#include <octomap/octomap_types.h>
#include <tf2/utils.h>
#include <random>
#include <roi_viewpoint_planner_msgs/VmpConfig.h>

namespace view_motion_planner
{

// Global configuration; reconfiguration managed by ViewMotionPlanner
inline VmpConfig config;

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
  double theta = z_dist(engine);
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

static octomap::point3d_collection computeVpRaycastEndpoints(const octomap::pose6d &vp)
{
  octomap::point3d_collection endpoints;
  const double hfov = 80 * M_PI / 180.0;
  const double vfov = 60 * M_PI / 180.0;
  const size_t x_steps = 9;
  const size_t y_steps = 7;
  const double maxRange = 1.0;

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

enum class EvalEpisodeEndParam
{
  TIME = 0,
  PLAN_DURATION = 1,
  PLAN_LENGTH = 2,
  NUM_EPEND_PARAMS = 3
};

} // namespace view_motion_planner
