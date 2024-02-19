#pragma once

// #include "drolib/base/shape_base.hpp"
#include "../base/shape_base.hpp"

namespace drolib {
class Ball : public ShapeBase {
 public:
  Ball();
  Ball(const Yaml &yaml);

  bool initialize() override;
  bool load(const Yaml &yaml) override;
  bool valid() const override;

  Eigen::VectorXd toD(const Eigen::Vector3d &p) const override;

  Eigen::Vector3d toP(const Eigen::VectorXd &d) const override;

  void getGradD(const Eigen::VectorXd &d, const Eigen::Matrix3Xd &gradP,
                const int i, const int j,
                Eigen::Map<Eigen::VectorXd> &gradD) const override;

 public:
  double radius{0.0};
  double margin{0.0};
};

}  // namespace drolib