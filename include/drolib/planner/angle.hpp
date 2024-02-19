#pragma once
#include "drolib/polynomial/cubic_spline.h"
#include <Eigen/Dense>

namespace drolib {
class AngleBase {
public:
  AngleBase() = default;
  virtual ~AngleBase() {}

  // virtual double at(const double t) const = 0;

  virtual Eigen::Vector3d at(const double t) const = 0;
};

class ConstAngle : public AngleBase {
public:
  ConstAngle() = default;
  ConstAngle(const double angle) : angle(angle) {}
  ~ConstAngle() {}

  // double at(const double t) const override {
  //   return angle;
  // }

  Eigen::Vector3d at(const double t) const override {
    return (Eigen::Vector3d() << angle, 0.0, 0.0).finished();
  }

  double angle{NAN};
};

// class SmoothAngle : public AngleBase {
//   SmoothAngle() = default;
//   SmoothAngle(const std::vector<double> &angles, const std::vector<double>
//   &timestamps)
//     : angles(angles), timestamps(timestamps) {
//     assert(angles.size() > 2);
//     assert(angles.size() == t.size());
//     angleSpline.set_points(timestamps, angles);
//   }
//   ~SmoothAngle() {}

//   double at(const double t) const override {
//     if (t < timestamps.front()) {
//       return angles.front();
//     } else if (t > timestamps.back()) {
//       return angles.back();
//     }
//     return angleSpline(t);
//   }

//   std::vector<double> angles;
//   std::vector<double> timestamps;
//   spline::CubicSpline angleSpline;
// };

} // namespace drolib