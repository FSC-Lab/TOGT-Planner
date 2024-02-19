#include "drolib/corridor/free_corridor.hpp"

namespace drolib {

FreeCorridor::FreeCorridor() {
  this->type = "FreeCorridor";
}

FreeCorridor::FreeCorridor(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                           const int midpoints, const double minDist)
    : start(p0), end(p1), midpoints(midpoints), minDist(minDist) {
  this->type = "FreeCorridor";
  std::shared_ptr<FreePoint> free = std::make_shared<FreePoint>();

  Eigen::Vector3d diff = end - start;
  Eigen::Vector3d gap = diff / (this->midpoints + 1);

  this->midpoints = gap.norm() < this->minDist
                        ? std::floor(diff.norm() / this->minDist)
                        : this->midpoints;
  gap = diff / (this->midpoints + 1);

  for (int i{0}; i < this->midpoints; ++i) {
    corridor.push_back(Waypoint(free));
    corridor.back().point = start + gap * (i + 1);
  }
}

FreeCorridor::FreeCorridor(const Yaml& yaml, const std::string& order) {
  this->type = "FreeCorridor";
  this->order = order;
  yaml["name"].getIfDefined(name);
  yaml["start"].getIfDefined(start);
  yaml["end"].getIfDefined(end);
  yaml["midpoints"].getIfDefined(midpoints);
  FreeCorridor(start, end, midpoints);
}

} // namespace drolib