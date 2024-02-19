#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include "drolib/base/parameter_base.hpp"

namespace drolib {

class QuadParams : public ParameterBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadParams() = default;

  using ParameterBase::load;
  bool load(const Yaml& yaml) override;
  bool valid() const override;
  friend std::ostream& operator<<(std::ostream& os, const QuadParams& params);

 public:
  std::string name{""};
  double mass{NAN};
  // double gravity{9.8066};
  Eigen::Vector3d inertia;
  // Eigen::Matrix4d T_mb;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T_mb;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T_bm;

  // Eigen::Matrix4d T_bm;
  // Eigen::Vector3d gravityVec;
};

}  // namespace drolib