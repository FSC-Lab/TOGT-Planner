#pragma once

#include <Eigen/Eigen>

namespace drolib {

struct Odom {
  Odom() = default;
  Odom(const Odom&) = default;
  ~Odom() = default;

  static constexpr int SIZE = 13;

  inline bool valid() const {
    return std::isfinite(t) &&
          position.allFinite() &&
          attitude.coeffs().allFinite() &&
          bodyvel.allFinite() &&
          bodyrate.allFinite();
  }

  inline bool reset() {
    t = NAN;
    position.setConstant(NAN);
    attitude.vec().setConstant(NAN); //TODO: not sure if it works
    bodyvel.setConstant(NAN);
    bodyrate.setConstant(NAN);
  }

  double t{NAN};
  Eigen::Vector3d position{NAN, NAN, NAN};
  Eigen::Quaterniond attitude{NAN, NAN, NAN, NAN};
  Eigen::Vector3d bodyvel{NAN, NAN, NAN};
  Eigen::Vector3d bodyrate{NAN, NAN, NAN};
};


}  // namespace drolib
