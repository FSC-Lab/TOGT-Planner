#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include "drolib/base/parameter_base.hpp"

namespace drolib {

class QuadParams : public ParameterBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadParams() {
    
    T_bm = (Eigen::Matrix4d() << 
                Eigen::Vector4d::Ones().transpose(), 
                Eigen::Vector4d(-1,  1, -1, 1).transpose(),
                Eigen::Vector4d(-1,  1, 1, -1).transpose(),
                Eigen::Vector4d(-1, -1, 1, 1).transpose()).finished();
    T_mb = T_bm.inverse();            
  }

  using ParameterBase::load;
  bool load(const Yaml& yaml) override;
  bool valid() const override;
  friend std::ostream& operator<<(std::ostream& os, const QuadParams& params);

 public:
  std::string name{""};
  double mass{1.0};
  // double gravity{9.8066};
  Eigen::Vector3d inertia{0.001, 0.001, 0.0017};
  // Eigen::Matrix4d T_mb;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T_mb;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T_bm;

  // Eigen::Matrix4d T_bm;
  // Eigen::Vector3d gravityVec;
};

}  // namespace drolib