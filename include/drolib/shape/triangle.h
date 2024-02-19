#pragma once

#include "drolib/shape/polygon.h"
#include "drolib/rotation/rotation_utils.h"

namespace drolib {

class Triangle : public Polygon {
 public:
  static constexpr int NUM_CORNERS = 3;
  using Polyhedron::initialize;
  
  Triangle() = default;
  Triangle(const Yaml &yaml, const double drift = 0.0);

  bool load(const Yaml &yaml) override;
  bool calVertices(const double drift = 0) override;

 public:
  double width{0.0};
  double height{0.0};
  double margin{0.0};
};
}  // namespace drolib