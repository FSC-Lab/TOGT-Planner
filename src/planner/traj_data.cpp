#include "drolib/planner/traj_data.hpp"
#include "drolib/planner/traj_solver.hpp"

namespace drolib {

TrajData& TrajData::operator=(const TrajData& other) {
  waypoints = other.waypoints;
  totalPieces = other.totalPieces;
  temporalVarDim = other.temporalVarDim;
  spatialVarDim = other.spatialVarDim;
  x = other.x;
  P = other.P;
  T = other.T;

  gradByPoints = other.gradByPoints;
  gradByTimes = other.gradByTimes;
  partialGradByCoeffs = other.partialGradByCoeffs;
  partialGradByTimes = other.partialGradByTimes;

  traj = other.traj;
  return *this;
}

bool TrajData::allocateSpace() {
  const int numPoints = getNumPoints();
  totalPieces =  numPoints + 1;
  temporalVarDim = totalPieces;
  spatialVarDim = 0;
  for (const auto& wp : waypoints) {
    spatialVarDim += wp.shape->dimension();
  }

  x.resize(temporalVarDim + spatialVarDim);
  P.resize(PATH_DIM, numPoints);
  T.resize(totalPieces);

  gradByPoints.resize(PATH_DIM, numPoints);
  gradByTimes.resize(totalPieces);
  partialGradByCoeffs.resize(NUM_COEFF * totalPieces, PATH_DIM);
  partialGradByTimes.resize(totalPieces);

  x.setZero();
  P.setZero();
  T.setZero();
  gradByPoints.setZero();
  gradByTimes.setZero();
  partialGradByCoeffs.setZero();
  partialGradByTimes.setZero();

  return true;
}

void TrajData::erase_front(const int n) {
  Eigen::MatrixXd Ttmp = T.bottomRows(T.rows() - n);
  T = Ttmp;
  P = P.rightCols(P.cols() - n);
  waypoints.erase(waypoints.begin(), waypoints.begin() + n);
  
  const int numPoints = getNumPoints();
  totalPieces =  numPoints + 1;
  temporalVarDim = totalPieces;
  spatialVarDim = 0;
  for (const auto& wp : waypoints) {
    spatialVarDim += wp.shape->dimension();
  }

  x.resize(temporalVarDim + spatialVarDim);
  calcInitialVal();

  gradByPoints.resize(PATH_DIM, numPoints);
  gradByTimes.resize(totalPieces);
  partialGradByCoeffs.resize(NUM_COEFF * totalPieces, PATH_DIM);
  partialGradByTimes.resize(totalPieces);

  traj.erase_front(n);
}

bool TrajData::initData(const Eigen::Vector3d &startPos,
                        const Eigen::Vector3d &endPos,
                        const double speedGuess) {
  if (!waypoints.empty()) {
    T(0) = averageTime(startPos, waypoints.front().point, speedGuess);
    int i{1};
    for (; i < getNumPoints(); ++i) {
      T(i) = averageTime(waypoints[i-1].point, waypoints[i].point, speedGuess);
    }
    T(i) = averageTime(waypoints.back().point, endPos, speedGuess);
  } else {
    T(0) = averageTime(startPos, endPos, speedGuess);
  }

  for (int j{0}; j < getNumPoints(); ++j) {
    P.col(j) = waypoints[j].point;
  }

  calcInitialVal();

  return true;
}

bool TrajData::calcInitialVal() {

  Eigen::Map<Eigen::VectorXd> K(x.data(), temporalVarDim);
  Eigen::Map<Eigen::VectorXd> D(x.data() + temporalVarDim, spatialVarDim);

  TrajSolver::backwardT(T, K);
  TrajSolver::backwardP(P, waypoints, D);
  
  // std::cout << "P before: \n" << P << "\n";

  // std::cout << "D after: \n" << D << "\n";

  // TrajSolver::forwardP(D, waypoints, P);
  // std::cout << "P after: \n" << P << "\n";

  return true;
}

bool TrajData::valid() const { return temporalVarDim > 0; }


void TrajData::backwardT(const Eigen::VectorXd &T,
                            Eigen::Map<Eigen::VectorXd> &K) {
  for (int i = 0; i < T.size(); i++) {
    K(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                      : (1.0 - sqrt(2.0 / T(i) - 1.0));
  }

  return;
}
void TrajData::backwardP(const Eigen::Matrix3Xd &P,
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


void TrajData::forwardT(const Eigen::VectorXd &K, Eigen::VectorXd &T) {
  for (int i = 0; i < K.size(); i++) {
    T(i) = K(i) > 0.0 ? ((0.5 * K(i) + 1.0) * K(i) + 1.0)
                      : 1.0 / ((0.5 * K(i) - 1.0) * K(i) + 1.0);
    // T(i) = K(i) * K(i);
  }
  return;
}

void TrajData::forwardP(const Eigen::VectorXd &D,
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


void TrajData::backPropagateT(const Eigen::VectorXd &K,
                                  const Eigen::VectorXd &gradT,
                                  Eigen::Map<Eigen::VectorXd> &gradK) {
  for (int i = 0; i < K.size(); i++) {
    if (K(i) > 0) {
      gradK(i) = gradT(i) * (K(i) + 1.0);
    } else {
      const double denSqrt = (0.5 * K(i) - 1.0) * K(i) + 1.0;
      gradK(i) = gradT(i) * (1.0 - K(i)) / (denSqrt * denSqrt);
    }
  }

  return;
}

void TrajData::backPropagateP(const Eigen::VectorXd &D,
                             const Eigen::Matrix3Xd &gradP,
                             const std::deque<Waypoint> &waypoints,
                             Eigen::Map<Eigen::VectorXd> &gradD) {
  int k{0}, l{0};
  int dim{0};
  for (const auto &wp : waypoints) {
    dim = wp.shape->dimension();
    wp.shape->getGradD(D.segment(l, dim), gradP, k, l, gradD);
    k++;
    l += dim;
  }
  return;
}

std::ostream& operator<<(std::ostream& os, const TrajData& data) {
  os.precision(4);
  // os << std::scientific;
  os << "TrajData:\n"
     << "spatialVarDim =    [" << data.spatialVarDim << "]\n"
     << "temporalVarDim =   [" << data.temporalVarDim << "]\n"
     << "totalPieces =      [" << data.totalPieces << "]\n"
     << "totalWaypoints =   [" << data.getNumPoints() << "]\n"
     << "Initial guesses:\n"
     << "P:\n [" << data.P.transpose() << "]\n"
     << "T:\n [" << data.T.transpose() << "]\n"
    //  << "x:\n [" << data.x.transpose() << "]"
     << std::endl;
  os.precision();
  // os.unsetf(std::ios::scientific);
  return os;
}

}  // namespace drolib