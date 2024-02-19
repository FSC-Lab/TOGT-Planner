#pragma once

#include "drolib/base/corridor_base.hpp"
#include "drolib/shape/free_point.h"

namespace drolib {

class FreeCorridor : public CorridorBase {
 public:
  FreeCorridor();

  FreeCorridor(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const int midpoints, const double minDist = 0.0);

  FreeCorridor(const Yaml& yaml, const std::string& order = "");

 public:
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  int midpoints{0};
  double minDist{0.0};
};

}