#pragma once

#include "drolib/planner/waypoint.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"
#include <mutex>

namespace drolib {

class CorridorBase {
public:
  std::vector<Waypoint> corridor;
  std::string order;
  std::string name;
  std::string type;
  std::mutex mutex;

  bool changed{false};

  CorridorBase() = default;

  CorridorBase(const std::string &order, const Yaml &yaml) {}

  explicit CorridorBase(const Yaml &yaml) {}

  virtual ~CorridorBase() {}

  virtual void getAllocation() {}

  virtual void write(std::ofstream &os) {}

  virtual Eigen::Vector3d getCenter() { return Eigen::Vector3d::Zero(); }

  virtual Eigen::Quaterniond getOrientation() {
    return Eigen::Quaterniond::Identity();
  }

  virtual Eigen::Vector3d getRotationEulerRPY() {
    return Eigen::Vector3d::Zero();
  }

  virtual double calcDistance(const Eigen::Vector3d &pos,
                              Eigen::Vector3d &gradDistanceToPos) {
    gradDistanceToPos.setZero();
    return 10000;
  }

  inline bool empty() const { return corridor.empty(); }

  inline int size() const { return corridor.size(); }

  inline bool isChanged() const { return changed; }

  inline void clearChange() { changed = false; }

  inline bool firstPoint(Eigen::Vector3d &point) const {
    if (empty()) {
      return false;
    }
    point = corridor.front().point;
    return true;
  }

  inline bool lastPoint(Eigen::Vector3d &point) const {
    if (empty()) {
      return false;
    }
    point = corridor.back().point;
    return true;
  }

  inline void reset() { corridor.clear(); }
};

} // namespace drolib