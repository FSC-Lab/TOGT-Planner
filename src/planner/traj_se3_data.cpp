#include "drolib/planner/traj_se3_data.hpp"

namespace drolib {

double TrajSE3Data::averageTime(const Eigen::Vector3d &p0,
                                const Eigen::Vector3d &p1,
                                const double speed) const {
  return (p1 - p0).norm() / speed;
}

bool TrajSE3Data::initData(const Eigen::Vector3d &startPos,
                           const Eigen::Vector3d &endPos,
                           const Eigen::Vector3d &object,
                           const double speedGuess) {
  if (!waypoints.empty()) {
    T(0) = averageTime(startPos, waypoints.front().point, speedGuess);
    int i{1};
    for (; i < getNumPoints(); ++i) {
      T(i) =
          averageTime(waypoints[i - 1].point, waypoints[i].point, speedGuess);
    }
    T(i) = averageTime(waypoints.back().point, endPos, speedGuess);
  } else {
    T(0) = averageTime(startPos, endPos, speedGuess);
  }

  for (int j{0}; j < getNumPoints(); ++j) {
    const Eigen::Vector3d &point = waypoints[j].point;
    const Eigen::Vector3d diff = (object - point);
    P.col(j) = point;
    Y.col(j) << std::atan2(diff.y(), diff.x());
    // Y.col(j) << 1.0;
  }

  calcInitialVal();

  return true;
}

bool TrajSE3Data::calcInitialVal() {

  Eigen::Map<Eigen::VectorXd> K(x.data(), temporalVarDim);
  Eigen::Map<Eigen::VectorXd> D(x.data() + temporalVarDim, spatialVarDim);
  Eigen::Map<Eigen::VectorXd> Z(x.data() + temporalVarDim + spatialVarDim, headingVarDim);

  backwardT(T, K);
  backwardP(P, waypoints, D);
  backwardY(Y, Z);

  return true;
}

bool TrajSE3Data::allocateSpace() {
  const int numPoints = getNumPoints();
  totalPieces = numPoints + 1;
  // Number of variables for trajectory time
  temporalVarDim = totalPieces;
  // Number of variables for heading
  headingVarDim = numPoints * 2;
  // Number of variables for spatial variables
  spatialVarDim = 0;
  for (const auto &wp : waypoints) {
    spatialVarDim += wp.shape->dimension();
  }

  x.resize(temporalVarDim + spatialVarDim + headingVarDim);
  T.resize(totalPieces);

  P.resize(PATH_DIM, numPoints);
  Y.resize(numPoints);
  gradByTimes.resize(totalPieces);
  gradByPoints.resize(PATH_DIM, numPoints);
  gradByYaw.resize(headingVarDim);
  partialGradByTimes.resize(totalPieces);
  partialGradByCoeffs.resize(NUM_COEFF * totalPieces, PATH_DIM);
  partialGradByHeadingCoeffs.resize(NUM_COEFF * totalPieces, YAW_DIM);

  x.setZero();
  T.setZero();
  P.setZero();
  Y.setZero();
  gradByPoints.setZero();
  gradByTimes.setZero();
  gradByYaw.setZero();
  partialGradByTimes.setZero();
  partialGradByCoeffs.setZero();
  partialGradByHeadingCoeffs.setZero();

  return true;
}

void TrajSE3Data::backwardT(const Eigen::VectorXd &T,
                            Eigen::Map<Eigen::VectorXd> &K) {
  for (int i = 0; i < T.size(); i++) {
    K(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                      : (1.0 - sqrt(2.0 / T(i) - 1.0));
  }

  return;
}
void TrajSE3Data::backwardP(const Eigen::Matrix3Xd &P,
                            const std::deque<Waypoint> &waypoints,
                            Eigen::Map<Eigen::VectorXd> &D) {
  int k{0}, l{0};
  int dim{0};
  for (const auto &wp : waypoints) {
    dim = wp.shape->dimension();
    D.segment(l, dim) = wp.shape->toD(P.col(k));
    k++;
    l += dim;
  }
}

void TrajSE3Data::backwardY(const Eigen::VectorXd &Y,
                            Eigen::Map<Eigen::VectorXd> &Z) {
  int k{0};
  for (int i = 0; i < Y.size(); i++) {
    // Z(i) = Y(i);
    Z.segment(i, 2) = Eigen::Vector2d(cos(Y(i)), sin(Y(i))); // (x=cos(\theta), y=sin(\theta))
    k += 2;
  }
}

void TrajSE3Data::forwardT(const Eigen::VectorXd &K, Eigen::VectorXd &T) {
  for (int i = 0; i < K.size(); i++) {
    T(i) = K(i) > 0.0 ? ((0.5 * K(i) + 1.0) * K(i) + 1.0)
                      : 1.0 / ((0.5 * K(i) - 1.0) * K(i) + 1.0);
    // T(i) = K(i) * K(i);
  }
  return;
}

void TrajSE3Data::forwardP(const Eigen::VectorXd &D,
                            const std::deque<Waypoint> &waypoints,
                            Eigen::Matrix3Xd &P) {
  int k{0}, l{0};
  int dim{0};
  for (const auto &wp :waypoints) {
    dim = wp.shape->dimension();
    P.col(k) = wp.shape->toP(D.segment(l, dim));
    k++;
    l += dim;
  }
  return;
}

void TrajSE3Data::forwardY(const Eigen::VectorXd &Z, Eigen::VectorXd &Y) {
  int k{0};
  for (int i = 0; i < static_cast<int>(Z.size() / 2); i++) {
    Y(i) = std::atan2(Z(k + 1), Z(k));
    k += 2;
  }
}


std::ostream& operator<<(std::ostream& os, const TrajSE3Data& data) {
  os.precision(4);
  // os << std::scientific;
  os << "TrajData:\n"
     << "temporalVarDim =   [" << data.temporalVarDim << "]\n"
     << "spatialVarDim =    [" << data.spatialVarDim << "]\n"
     << "headingVarDim =    [" << data.headingVarDim << "]\n"
     << "totalPieces =      [" << data.totalPieces << "]\n"
     << "totalWaypoints =   [" << data.getNumPoints() << "]\n"
     << "Initial guesses:\n"
     << "T:\n [" << data.T.transpose() << "]\n"
     << "P:\n [" << data.P.transpose() << "]\n"
     << "Y:\n [" << data.Y.transpose() << "]\n"
    //  << "x:\n [" << data.x.transpose() << "]"
     << std::endl;
  os.precision();
  // os.unsetf(std::ios::scientific);
  return os;
}

} // namespace drolib