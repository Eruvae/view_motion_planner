#pragma once

#include <octomap/octomap_types.h>

namespace view_motion_planner
{

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

} // namespace view_motion_planner
