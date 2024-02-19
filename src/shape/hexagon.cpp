#include "drolib/shape/hexagon.h"

namespace drolib {

Hexagon::Hexagon(const Yaml &yaml, const double drift) {
  load(yaml);
  calVertices(drift);
  initialize();
}  

bool Hexagon::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Hexagon";
  this->shape = Shape::HEXAGON;

  Eigen::Vector3d rpy;
  yaml["position"].getIfDefined(position);
  yaml["rpy"].getIfDefined(rpy);
  yaml["side"].getIfDefined(side);
  yaml["margin"].getIfDefined(margin);
  R_wb = eulerAnglesRPYToQuaternion(deg2rad(rpy));

  return true;
}

bool Hexagon::calVertices(const double drift) {
  double aside = side - margin;
  double hside = 0.5 * aside;
  double height = hside * tan(M_PI / 3.0);

  Eigen::Vector3d c0, c1, c2, c3, c4, c5;
  c0 << -height, hside, drift;
  c1 << 0.0, aside, drift;
  c2 << height, hside, drift;
  c3 << height, -hside, drift;
  c4 << 0.0, -aside, drift;
  c5 << -height, -hside, drift;

  vertices.resize(NUM_CORNERS);
  vertices[0] = R_wb * c0 + position;
  vertices[1] = R_wb * c1 + position;
  vertices[2] = R_wb * c2 + position;
  vertices[3] = R_wb * c3 + position;
  vertices[4] = R_wb * c4 + position;
  vertices[5] = R_wb * c5 + position;
  
  return true;
}

}  // namespace drolib