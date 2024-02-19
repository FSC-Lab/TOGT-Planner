#pragma once

#include "drolib/shape/polygon.h"
#include "drolib/rotation/rotation_utils.h"

namespace drolib {

class Hexagon : public Polygon {
 public:
  static constexpr int NUM_CORNERS = 6;
  using Polyhedron::initialize;

  Hexagon() = default;
  Hexagon(const Yaml &yaml, const double drift = 0.0);

  bool load(const Yaml &yaml) override;
  bool calVertices(const double drift = 0) override;

 public:
  double side{0.0};
  double margin{0.0};
};
}  // namespace drolib