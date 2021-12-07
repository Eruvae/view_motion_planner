#pragma once

#include <octomap/octomap_types.h>
#include <tf2/utils.h>

namespace view_motion_planner
{

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

} // namespace view_motion_planner
