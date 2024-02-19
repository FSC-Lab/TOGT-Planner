#pragma once

#include "drolib/base/shape_base.hpp"

namespace drolib {

class FreePoint : public ShapeBase {
 public:
  FreePoint();
  FreePoint(const Yaml &yaml);
  
  bool initialize() override;
  bool load(const Yaml &yaml) override;
  bool valid() const override;

  inline Eigen::VectorXd toD(const Eigen::Vector3d &p) const override {
    return p;
  }

  inline Eigen::Vector3d toP(const Eigen::VectorXd &d) const override {
    return d;
  }

  inline void getGradD(const Eigen::VectorXd &d, const Eigen::Matrix3Xd &gradP,
                       const int i, const int j,
                       Eigen::Map<Eigen::VectorXd> &gradD) const override {
    gradD.segment(j, dim) = gradP.col(i);
    return;
  }
};

}  // namespace drolib