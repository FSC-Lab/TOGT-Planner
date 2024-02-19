#pragma once

#include "drolib/shape/polyhedron.h"
#include "drolib/rotation/rotation_utils.h"

namespace drolib {

class Polygon : public Polyhedron {
 public:
  using Polyhedron::initialize;

  Polygon() = default;
  Polygon(const Yaml &yaml);
  virtual bool calVertices(const double drift = 0);

  bool load(const Yaml &yaml) override;
 public:
  Eigen::Quaterniond R_wb;
};

}  // namespace drolib