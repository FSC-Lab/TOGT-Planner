#pragma once

#include <Eigen/Eigen>
#include "drolib/base/shape_base.hpp"

namespace drolib {

struct Waypoint {
  Waypoint() = default;
  Waypoint(std::shared_ptr<ShapeBase> obs) : shape(obs) {
    point = obs->position;
  }
  std::shared_ptr<ShapeBase> shape;
  Eigen::Vector3d point;
};

}  // namespace drolib