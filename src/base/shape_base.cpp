#include "drolib/base/shape_base.hpp"

namespace drolib {

bool ShapeBase::load(const Yaml &yaml) {
  return false;
}

bool ShapeBase::initialize() {
  dim = 0;
  return false;
}

bool ShapeBase::valid() const { return false; }

}  // namespace drolib