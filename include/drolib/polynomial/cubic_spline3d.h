#pragma once

#include "cubic_spline.h"
#include <Eigen/Dense>

namespace drolib {

namespace spline {

class CubicSpline3D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CubicSpline3D() {}
  ~CubicSpline3D() {}

  bool isInPath(const double s);

  void plan(const std::vector<Eigen::Vector3d>& points,
            const std::vector<double>& s_vec);
            
  // PolynomialTrajectory coeffs(const double s);

  inline Eigen::Vector3d position(const double s) const {
    double x = x_(s);
    double y = y_(s);
    double z = z_(s);
    return Eigen::Vector3d(x, y, z);
  };

  inline Eigen::Vector3d tangent(const double s) const {
    double dx = x_.deriv(1, s);
    double dy = y_.deriv(1, s);
    double dz = z_.deriv(1, s);
    return Eigen::Vector3d(dx, dy, dz);
  };

  inline bool isValid() const { return valid_; }

 private:
  std::vector<double> s_vec_;
  CubicSpline x_;
  CubicSpline y_;
  CubicSpline z_;
  bool valid_;
};

}

}  // namespace drolib
