#include "drolib/shape/triangle.h"

namespace drolib {

Triangle::Triangle(const Yaml &yaml, const double drift) {
  load(yaml);
  calVertices(drift);
  initialize();
}  

bool Triangle::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Triangle";
  this->shape = Shape::TRIANGLE;

  Eigen::Vector3d rpy;
  yaml["position"].getIfDefined(position);
  yaml["rpy"].getIfDefined(rpy);
  yaml["width"].getIfDefined(width);
  yaml["height"].getIfDefined(height);
  yaml["margin"].getIfDefined(margin);

  R_wb = eulerAnglesRPYToQuaternion(deg2rad(rpy));

  return true;
}

bool Triangle::calVertices(const double drift) {
  const double hw = 0.5 * (width - margin);
  const double hh = 0.5 * (height - margin);

  // counter-clock wise starting from the bottom left corner
  Eigen::Vector3d c0, c1, c2;
  c0 << -hh, hw, drift;
  c1 <<  hh, 0, drift;
  c2 << -hh, -hw, drift;

  vertices.resize(NUM_CORNERS);
  vertices[0] = R_wb * c0 + position;
  vertices[1] = R_wb * c1 + position;
  vertices[2] = R_wb * c2 + position;
  
  return true;
}

}  // namespace drolib