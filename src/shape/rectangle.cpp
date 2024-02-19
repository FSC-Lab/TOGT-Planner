#include "drolib/shape/rectangle.h"

namespace drolib {

Rectangle::Rectangle(const Yaml &yaml, const double drift) {
  load(yaml);
  calVertices(drift);
  initialize();
}

bool Rectangle::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Rectangle";
  this->shape = Shape::RECTANGLE;
  Eigen::Vector3d rpy;
  yaml["position"].getIfDefined(position);
  yaml["rpy"].getIfDefined(rpy);
  yaml["width"].getIfDefined(width);
  yaml["height"].getIfDefined(height);
  yaml["marginW"].getIfDefined(marginW);
  yaml["marginH"].getIfDefined(marginH);

  R_wb = eulerAnglesRPYToQuaternion(deg2rad(rpy));

  return true;
}

bool Rectangle::calVertices(const double drift) {
  const double hw = 0.5 * (width - marginW);
  const double hh = 0.5 * (height - marginH);

  // counter-clock wise starting from the bottom left corner
  Eigen::Vector3d c0, c1, c2, c3;
  c0 << -hh, hw, drift;
  c1 << -hh, -hw, drift;
  c2 << hh, -hw, drift;
  c3 << hh, hw, drift;

  vertices.resize(NUM_CORNERS);
  vertices[0] = R_wb * c0 + position;
  vertices[1] = R_wb * c1 + position;
  vertices[2] = R_wb * c2 + position;
  vertices[3] = R_wb * c3 + position;

  return true;
}

}  // namespace drolib