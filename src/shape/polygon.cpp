#include "drolib/shape/polygon.h"

namespace drolib {

Polygon::Polygon(const Yaml &yaml) {
  load(yaml);
  calVertices();
  initialize();
}  

bool Polygon::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Polygon";
  this->shape = Shape::POLYGON;

  Eigen::Vector3d rpy;
  yaml["position"].getIfDefined(position);
  yaml["rpy"].getIfDefined(rpy);
  R_wb = eulerAnglesRPYToQuaternion(deg2rad(rpy));

  // std::vector<Eigen::Vector3d> corners;
  // yaml["corners"].getIfDefined(corners);
  // for (const auto& c : corners) {
  //   std::cout << c.transpose() << "\n";
  // }

  return true;
}

bool Polygon::calVertices(const double drift) {
  return false;
}

}  // namespace drolib