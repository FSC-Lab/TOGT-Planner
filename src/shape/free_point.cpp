#include "drolib/shape/free_point.h"

namespace drolib {

FreePoint::FreePoint() {
  this->name = "Free";
  this->shape = Shape::FREEPOINT;
  initialize();
}

FreePoint::FreePoint(const Yaml &yaml) {
  load(yaml);
  initialize();
}  

bool FreePoint::initialize() {
  dim = DIM3;
  return true;
}

bool FreePoint::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Free";
  this->shape = Shape::FREEPOINT;
  // yaml["position"].getIfDefined(position);
  return true;
}


//TODO:
bool FreePoint::valid() const {
  return (dim == DIM3);
}

}  // namespace drolib