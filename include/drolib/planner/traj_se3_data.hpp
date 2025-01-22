#ifndef DROLIB_PLANNER_TRAJ_SE3_DATA_HPP_
#define DROLIB_PLANNER_TRAJ_SE3_DATA_HPP_

#include "drolib/planner/waypoint.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"
#include "drolib/type/types.hpp"
#include <Eigen/Eigen>
#include <deque>
#include <memory>

namespace drolib {

struct TrajSE3Data {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TrajSE3Data() = default;

  friend std::ostream &operator<<(std::ostream &os, const TrajSE3Data &data);

  bool allocateSpace();

  bool initData(const Eigen::Vector3d &startPos, const Eigen::Vector3d &endPos,
                const Eigen::Vector3d &object, const double speedGuess = 1.0);

  double averageTime(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                     const double speed) const;

  bool calcInitialVal();

  static void backwardT(const Eigen::VectorXd &T,
                        Eigen::Map<Eigen::VectorXd> &K);

  static void backwardP(const Eigen::Matrix3Xd &P,
                        const std::deque<Waypoint> &waypoints,
                        Eigen::Map<Eigen::VectorXd> &D);

  static void backwardY(const Eigen::VectorXd &Y,
                        Eigen::Map<Eigen::VectorXd> &Z);

  static void forwardT(const Eigen::VectorXd &K, Eigen::VectorXd &T);

  static void forwardP(const Eigen::VectorXd &D,
                       const std::deque<Waypoint> &waypoints,
                       Eigen::Matrix3Xd &P);

  static void forwardY(const Eigen::VectorXd &Z, Eigen::VectorXd &Y);

  inline int getNumPoints() const { return waypoints.size(); }

  inline void clear() {
    totalPieces = 0;
    temporalVarDim = 0;
    spatialVarDim = 0;
    headingVarDim = 0;
    waypoints.clear();
  }

  inline void append(const std::vector<Waypoint> &points) {
    waypoints.insert(waypoints.end(), points.begin(), points.end());
  }

  std::deque<Waypoint> waypoints;

  int totalPieces{0};
  int temporalVarDim{0};
  int spatialVarDim{0};
  int headingVarDim{0};

  Eigen::VectorXd x;
  
  Eigen::VectorXd T;
  Eigen::Matrix3Xd P;
  Eigen::VectorXd Y;

  Eigen::VectorXd gradByTimes;
  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByYaw;
  Eigen::VectorXd partialGradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByHeadingCoeffs;
};

} // namespace drolib

#endif  /* DROLIB_PLANNER_TRAJ_SE3_DATA_HPP_ */
