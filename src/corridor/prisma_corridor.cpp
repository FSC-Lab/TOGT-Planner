#include "drolib/corridor/prisma_corridor.hpp"

namespace drolib {

PrismaCorridor::PrismaCorridor() {
  this->type = "Prisma";
}

PrismaCorridor::PrismaCorridor(const std::string shape, const Yaml& yaml, const std::string& order) {
  this->type = "Prisma";
  this->order = order;
  yaml["name"].getIfDefined(name);
  yaml["length"].getIfDefined(length);
  yaml["midpoints"].getIfDefined(midpoints);

  shapeName = shape.substr(0, shape.find(this->type));
  if (length > 1.0e-3) {

    std::shared_ptr<Polygon> in = createShape.at(shapeName)(yaml, 0.5 * length);
    std::shared_ptr<Polygon> out = createShape.at(shapeName)(yaml, -0.5 * length);

    corridor.reserve(midpoints + 2);
    corridor.push_back(Waypoint(in));
    if (midpoints > 0) {
      std::vector<Eigen::Vector3d> vertices;
      vertices.insert(vertices.end(), in->vertices.begin(),
                      in->vertices.end());
      vertices.insert(vertices.end(), out->vertices.begin(),
                      out->vertices.end());
      std::shared_ptr<Polyhedron> mid =
          std::make_shared<Polyhedron>(vertices);

      const Eigen::Vector3d &p0 = in->position;
      const Eigen::Vector3d &p1 = out->position;

      const int numPieces = midpoints + 1;
      Eigen::Vector3d gap = (p1 - p0) / numPieces;
      for (int i{0}; i < midpoints; ++i) {
        corridor.push_back(Waypoint(mid));
        corridor.back().point = p0 + gap * (i + 1);
      }
    }

    corridor.push_back(Waypoint(out));
  } else {
    corridor.emplace_back(createShape.at(shapeName)(yaml, 0.0));
  }

}

// PrismaCorridor::PrismaCorridor(const std::string shape, const Yaml& yaml) {
//   PrismaCorridor("", yaml);
// }

} // namespace drolib