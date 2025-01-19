#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include "drolib/base/parameter_base.hpp"
#include "drolib/rotation/rotation_utils.h"

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

    q_bc = eulerAnglesRPYToQuaternion(deg2rad(Eigen::Vector3d(-60, 0, -90)));
    gate_orient = eulerAnglesRPYToQuaternion(deg2rad(Eigen::Vector3d(-90, 0, -90)));

    std::cout << "q_bc: " << q_bc.coeffs().transpose() << std::endl;
    std::cout << "gate_orient: " << gate_orient.coeffs().transpose() << std::endl;

    const Eigen::Quaterniond q_wb{Eigen::Quaterniond::Identity()};
    const Eigen::Quaterniond q_wc = q_wb * q_bc;
    const Eigen::Vector3d z_c = q_wc.toRotationMatrix().col(2);
    const Eigen::Vector3d z_g = gate_orient.toRotationMatrix().col(2);
    std::cout << "z_c: " << z_c.transpose() << std::endl;
    std::cout << "z_g: " << z_g.transpose() << std::endl;

    double cost = 1.0 - z_c.dot(z_g);
    std::cout << "cost: " << cost << std::endl;

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

  Eigen::Quaterniond gate_orient{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond q_bc{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d t_b_cb{Eigen::Vector3d::Zero()};

};

}  // namespace drolib