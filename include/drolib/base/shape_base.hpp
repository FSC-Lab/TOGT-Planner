#pragma once

#include <Eigen/Eigen>
#include <exception>
#include "drolib/file/yaml.hpp"

namespace drolib {

enum class Shape : unsigned char {
  FREEPOINT = 0,
  BALL = 1,
  POLYHEDRON = 2,
  POLYGON = 3,
  HEXAGON = 4,
  PENTAGON = 5,
  RECTANGLE = 6,
  SQUARE = 7,
  TRIANGLE = 8,
  CIRCLE = 9,
  ELLIPSE = 10,
  UNDEFINED = 11
};

class ShapeBase {
 public:
  static constexpr int DIM3 = 3; 
  ShapeBase() = default;
  virtual ~ShapeBase() {}
  virtual Eigen::VectorXd toD(const Eigen::Vector3d &p) const = 0;
  virtual Eigen::Vector3d toP(const Eigen::VectorXd &d) const = 0;
  virtual void getGradD(const Eigen::VectorXd &d, const Eigen::Matrix3Xd &gradP,
                        const int i, const int j,
                        Eigen::Map<Eigen::VectorXd> &gradD) const = 0;  
  virtual bool initialize();                   
  virtual bool load(const Yaml &yaml);
  virtual bool valid() const;
  inline int dimension(void) const { return dim; }

  std::string name;
  Shape shape;
  Eigen::Vector3d position;
  int dim{-1};
};

}  // namespace drolib