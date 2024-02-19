#pragma once

#include <cfloat>
#include "drolib/base/shape_base.hpp"
#include "drolib/solver/lbfgs.hpp"
namespace drolib {

class Polyhedron : public ShapeBase {
 public:
  static constexpr int MAX_ITERATION = 128;
  static constexpr double RES_TOLERANCE = 1.0e-5;

  using PolyhedronV = Eigen::Matrix3Xd;
  using PolyhedronH = Eigen::MatrixX4d;

  Polyhedron() = default;
  Polyhedron(const Yaml &yaml);
  Polyhedron(const std::vector<Eigen::Vector3d> &vertices);

  bool initialize() override;
  bool load(const Yaml &yaml) override;
  bool valid() const override;

  Eigen::VectorXd toD(const Eigen::Vector3d &p) const override;

  Eigen::Vector3d toP(const Eigen::VectorXd &d) const override;

  void getGradD(const Eigen::VectorXd &d, const Eigen::Matrix3Xd &gradP,
                const int i, const int j,
                Eigen::Map<Eigen::VectorXd> &gradD) const override;

  static double costTinyNLS(void *ptr, const Eigen::VectorXd &d,
                            Eigen::VectorXd &gradD);

 public:
  PolyhedronV coordinate;
  PolyhedronV polyhedron;
  std::vector<Eigen::Vector3d> vertices;

 public:
  lbfgs_parameter_t lbfgsParams;
};

}  // namespace drolib