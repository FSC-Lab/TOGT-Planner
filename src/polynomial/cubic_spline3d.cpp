#include "drolib/polynomial/cubic_spline3d.h"

namespace drolib {

namespace spline {

bool CubicSpline3D::isInPath(const double s) {
  if (!isValid()) {
    return false;
  }

  if (s < s_vec_.front() || s > s_vec_.back()) {
    return false;
  }

  return true;
}

void CubicSpline3D::plan(const std::vector<Eigen::Vector3d>& points,
                         const std::vector<double>& s_vec) {
  if (points.size() != s_vec.size()) {
    // std::cout << "CubicSpline3D: Sample size does not match." << std::endl;
    valid_ = false;
    return;
  }

  std::vector<double> vx, vy, vz;
  for (const auto& point : points) {
    vx.push_back(point.x());
    vy.push_back(point.y());
    vz.push_back(point.z());
  }

  x_.set_points(s_vec, vx);
  y_.set_points(s_vec, vy);
  z_.set_points(s_vec, vz);
  s_vec_ = s_vec;

  valid_ = true;
}

// PolynomialTrajectory CubicSpline3D::coeffs(const double s) {
//   if (!isValid()) {
//     return PolynomialTrajectory();
//   }

//   PolynomialTrajectory traj(x_.coeffs(s), y_.coeffs(s), z_.coeffs(s));

//   return traj;
// }

}
}  // namespace drolib
