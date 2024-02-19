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