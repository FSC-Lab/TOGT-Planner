#pragma once

#include <Eigen/Eigen>

namespace drolib {

struct ImuSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuSample() = default;
  ImuSample(const ImuSample&) = default;

  ImuSample(const double t, const Eigen::Vector3d& acc,
            const Eigen::Vector3d& omega)
      : t(t), acc(acc), omega(omega) {}

  ~ImuSample() = default;

  inline bool valid() const {
    return omega.allFinite() && acc.allFinite() && std::isfinite(t);
  }

  double t{NAN};
  Eigen::Vector3d acc{NAN, NAN, NAN};
  Eigen::Vector3d omega{NAN, NAN, NAN};

  inline bool operator<(const double time) const { return t < time; }
  inline bool operator<=(const double time) const { return t <= time; }
  inline bool operator>(const double time) const { return t > time; }
  inline bool operator>=(const double time) const { return t >= time; }
};

}  // namespace drolib