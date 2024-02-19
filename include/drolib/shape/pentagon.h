#pragma once

#include "drolib/shape/polygon.h"
#include "drolib/rotation/rotation_utils.h"

namespace drolib {

class Pentagon : public Polygon {
 public:
  static constexpr int NUM_CORNERS = 5;
  using Polyhedron::initialize;

  Pentagon() = default;
  Pentagon(const Yaml &yaml, const double drift = 0.0);

  bool load(const Yaml &yaml) override;
  bool calVertices(const double drift = 0) override;

 public:
  double radius{0.0};
  double margin{0.0};
};
}  // namespace drolib