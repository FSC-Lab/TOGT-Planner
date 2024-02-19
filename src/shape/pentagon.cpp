#include "drolib/shape/pentagon.h"

namespace drolib {

Pentagon::Pentagon(const Yaml &yaml, const double drift) {
  load(yaml);
  calVertices(drift);
  initialize();
}  

bool Pentagon::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;
  
  this->name = "Pentagon";
  this->shape = Shape::PENTAGON;

  Eigen::Vector3d rpy;
  yaml["position"].getIfDefined(position);
  yaml["rpy"].getIfDefined(rpy);
  yaml["radius"].getIfDefined(radius);
  yaml["margin"].getIfDefined(margin);
  R_wb = eulerAnglesRPYToQuaternion(deg2rad(rpy));

  return true;
}

bool Pentagon::calVertices(const double drift) {
  double ar = radius - margin;
  double cos54 = cos(0.3 * M_PI);
  double sin54 = sin(0.3 * M_PI);
  double nd = ar * cos54;
  double on = ar * sin54;
  double bc = 2 * nd;
  double fc = bc * sin54;
  double of = ar - bc * cos54;

  Eigen::Vector3d c0, c1, c2, c3, c4;
  c0 << -on, nd, drift;
  c1 << of, fc, drift;
  c2 << ar, 0.0, drift;
  c3 << of, -fc, drift;
  c4 << -on, -nd, drift;

  vertices.resize(NUM_CORNERS);
  vertices[0] = R_wb * c0 + position;
  vertices[1] = R_wb * c1 + position;
  vertices[2] = R_wb * c2 + position;
  vertices[3] = R_wb * c3 + position;
  vertices[4] = R_wb * c4 + position;
  
  return true;
}

}  // namespace drolib