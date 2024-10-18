#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <deque>
#include "drolib/type/types.hpp"
#include "drolib/planner/waypoint.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"

namespace drolib {

struct TrajData {
  TrajData() = default;

  bool allocateSpace();

  bool initData(const Eigen::Vector3d &startPos,
                const Eigen::Vector3d &endPos,
                const double speedGuess = 1.0);

  friend std::ostream& operator<<(std::ostream& os, const TrajData& data);

  bool valid() const;

  TrajData& operator=(const TrajData& other);

  inline int getNumPoints() const { return waypoints.size(); }


  inline void clear() {
    totalPieces = 0;
    temporalVarDim = 0;
    spatialVarDim = 0;
    waypoints.clear();
  }

  inline void append(const std::vector<Waypoint> &points) {
    waypoints.insert(waypoints.end(), points.begin(), points.end());
  }

  inline double averageTime(const Eigen::Vector3d &p0,
                            const Eigen::Vector3d &p1,
                            const double speed) const {
    return (p1 - p0).norm() / speed;   
  }

  void erase_front(const int n);
  
  bool calcInitialVal();

  std::deque<Waypoint> waypoints;
  int totalPieces{0};
  int spatialVarDim{0};
  int temporalVarDim{0};

  Eigen::VectorXd x;
  Eigen::Matrix3Xd P;
  Eigen::VectorXd T;

  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByTimes;

  PiecewisePolynomial<POLY_DEG> traj;
};

}  // namespace drolib
