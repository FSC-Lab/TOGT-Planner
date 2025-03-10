#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <deque>
#include "drolib/type/types.hpp"
#include "drolib/planner/waypoint.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"

#include "drolib/base/corridor_base.hpp"
#include "drolib/corridor/prisma_corridor.hpp"
#include "drolib/corridor/single_ball.hpp"

namespace drolib {

struct TrajData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TrajData() = default;

  bool allocateSpace();

  bool initData(const Eigen::Vector3d &startPos,
                const Eigen::Vector3d &endPos,
                const double speedGuess = 1.0);

  friend std::ostream& operator<<(std::ostream& os, const TrajData& data);

  bool valid() const;

  void setGates(std::vector<std::shared_ptr<CorridorBase>> gate_from_track) { gates = gate_from_track; }

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

  std::vector<std::shared_ptr<CorridorBase>> gates;
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

  static void backwardT(const Eigen::VectorXd &T,
                        Eigen::Map<Eigen::VectorXd> &K);

  static void backwardP(const Eigen::Matrix3Xd &P,
                        const std::deque<Waypoint> &waypoints,
                        Eigen::Map<Eigen::VectorXd> &D);

  static void forwardT(const Eigen::VectorXd &K, Eigen::VectorXd &T);

  static void forwardP(const Eigen::VectorXd &D,
                       const std::deque<Waypoint> &waypoints,
                       Eigen::Matrix3Xd &P);
  static void backPropagateT(const Eigen::VectorXd &K,
                             const Eigen::VectorXd &gradT,
                             Eigen::Map<Eigen::VectorXd> &gradK);

  static void backPropagateP(const Eigen::VectorXd &D,
                             const Eigen::Matrix3Xd &gradP,
                             const std::deque<Waypoint> &waypoints,
                             Eigen::Map<Eigen::VectorXd> &gradD);

};

}  // namespace drolib
