#pragma once

#include "drolib/shape/polygon.h"
#include "drolib/rotation/rotation_utils.h"

namespace drolib {

class Rectangle : public Polygon {
 public:
  static constexpr int NUM_CORNERS = 4;
  using Polyhedron::initialize;

  Rectangle() = default;
  Rectangle(const Yaml &yaml, const double drift = 0.0);
  bool load(const Yaml &yaml) override;
  bool calVertices(const double drift = 0) override;

 public:
  double width{0.0};
  double height{0.0};
  double marginW{0.0};
  double marginH{0.0};


};
}  // namespace drolib