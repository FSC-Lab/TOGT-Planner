#include "drolib/planner/traj_se3_solver.hpp"

namespace drolib {

bool TrajSE3Solver::solve(const PVAJ &initState,
            const PVAJ &endState, 
            const Eigen::Vector3d &initYaw,
            const Eigen::Vector3d &endYaw,
            const QuadManifold &quad,
            const TrajParams &tparams,
            const LbfgsParams& lbfgs) {
  if (!data.valid()) {
    std::cout << "Data is not initialized properly.\n";
    return false;
  }


  this->quad = quad; 
  this->tparams = tparams; 

  minco.setConditions(initState, endState, data.totalPieces);
  minco_yaw.setConditions(initYaw.T, endYaw.T, data.totalPieces);

  double cost{0.0};
  int ret = lbfgs_optimize(data.x, cost, &TrajSE3Solver::costFunction, nullptr, nullptr, this, lbfgs.params);

  // std::cout << "minimum cost: " << cost << "\n";
  if (ret < 0) {
    status = Status::LBFGS_FAILED;
    return false;
  }

  int total_dim = data.temporalVarDim + data.spatialVarDim;


  Eigen::Map<const Eigen::VectorXd> K(data.x.data(), data.temporalVarDim);
  Eigen::Map<const Eigen::VectorXd> D(data.x.data() + data.temporalVarDim, data.spatialVarDim);
  forwardT(K, data.T);
  forwardP(D, data.waypoints, data.P);


  status = static_cast<Status>(ret);
  minco.getTrajectory(data.traj);


  return true;
}

double TrajSE3Solver::costFunction(void *ptr, const Eigen::VectorXd &x,
                                  Eigen::VectorXd &g) {
  TrajSE3Solver *obj = reinterpret_cast<TrajSE3Solver *>(ptr);
  const int dimK = obj->data.temporalVarDim;
  const int dimD = obj->data.spatialVarDim;

  // Obtain current variables with gradients
  Eigen::Map<const Eigen::VectorXd> K(x.data(), dimK);
  Eigen::Map<const Eigen::VectorXd> D(x.data() + dimK, dimD);
  Eigen::Map<Eigen::VectorXd> gradK(g.data(), dimK);
  Eigen::Map<Eigen::VectorXd> gradD(g.data() + dimK, dimD);

  forwardT(K, obj->data.T);
  forwardP(D, obj->data.waypoints, obj->data.P);

  double cost{0.0};
  obj->data.partialGradByCoeffs.setZero();
  obj->data.partialGradByTimes.setZero();

  obj->minco.setParameters(obj->data.P, obj->data.T);

  cost += addEnergyCost(obj->minco, obj->tparams, obj->data.partialGradByCoeffs, obj->data.partialGradByTimes);
  cost += addPenaltyCost(obj->data.T, obj->minco.getCoeffs(), obj->quad, obj->yawTilt.get(), obj->tparams, obj->data.partialGradByCoeffs, obj->data.partialGradByTimes);

  obj->minco.propagateGrad(obj->data.partialGradByCoeffs, obj->data.partialGradByTimes, obj->data.gradByPoints, obj->data.gradByTimes);

  cost += addTimeCost(obj->data.T, obj->tparams, obj->data.gradByTimes);

  backPropagateT(K, obj->data.gradByTimes, gradK);
  backPropagateP(D, obj->data.gradByPoints, obj->data.waypoints, gradD);

  return cost;
}

double TrajSE3Solver::addEnergyCost(const MincoSnap &minco,
                                    const TrajParams &params,
                                    Eigen::MatrixX3d &gradC,
                                    Eigen::VectorXd &gradT) {
  double cost{0.0};
  if (params.weightEnergy <= 1.0e-6) {
    return cost;
  }                           
  minco.getEnergyWithGrads(cost, gradC, gradT);
  cost *= params.weightEnergy;
  gradC *= params.weightEnergy;
  gradT *= params.weightEnergy;
  return cost;
}

double TrajSE3Solver::addTimeCost(const Eigen::VectorXd &T,
                                  const TrajParams &params,
                                  Eigen::VectorXd &gradByTimes) {
  gradByTimes.array() += params.weightTime;
  return params.weightTime * T.sum();
}


double TrajSE3Solver::addPenaltyCost(const Eigen::VectorXd &T,
                                    const Eigen::MatrixX3d &coeffs,
                                    const QuadManifold &quad,
                                    AngleBase* yawTilt,
                                    const TrajParams &params,       
                                    Eigen::MatrixX3d &gradC,
                                    Eigen::VectorXd &gradT) {
  double cost{0.0};
  double step, alpha;
  double node, penalty;
  Eigen::Matrix<double, NUM_COEFF, 6> beta;
  Eigen::Vector3d pos, vel, acc, jer, sna, cra;
  Eigen::Vector3d yaw;
  PVAJS pvajs;
  Setpoint setpoint;

  Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc;
  Eigen::Vector3d totalGradJer, totalGradSna, totalGradCra;
  Eigen::Vector3d totalGradHeading;
  Eigen::Vector3d gradPos;
  Eigen::Vector3d gradVel;
  Eigen::Vector3d gradOmg;
  Eigen::Vector4d gradQuat;
  Eigen::Vector4d gradThrusts;

  const int pieceNum = T.size();
  int numCheckPerPiece = params.numConstPena;

  for (int i = 0; i < pieceNum; i++) {
    const Eigen::Matrix<double, NUM_COEFF, PATH_DIM> &c = coeffs.block<NUM_COEFF, PATH_DIM>(i * NUM_COEFF, 0);
    // if (T(i) <= params.checkTimeSec) {
    //   continue;
    // }
    if (params.dynamicConstCheck) {
      numCheckPerPiece = T(i) / params.checkTimeSec;
      numCheckPerPiece = numCheckPerPiece <= params.maxNumCheck ? numCheckPerPiece : params.maxNumCheck;
      numCheckPerPiece = numCheckPerPiece < params.minNumCheck ? params.minNumCheck : numCheckPerPiece;
    }

    const double integralFrac = 1.0f / numCheckPerPiece;
    step = T(i) * integralFrac;
    for (int j = 0; j <= numCheckPerPiece; j++) {
      computeBeta(j * step, beta);
      pos = c.transpose() * beta.col(0);
      vel = c.transpose() * beta.col(1);
      acc = c.transpose() * beta.col(2);
      jer = c.transpose() * beta.col(3);
      sna = c.transpose() * beta.col(4);
      cra = c.transpose() * beta.col(5);

      //TODO: compute yaw angle from another polynomial
      yaw = yawTilt->at(j * step);

      pvajs << pos, vel, acc, jer, sna;
      penalty = 0.0;

      /***********************************************/
      // penalty += quad.computePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      penalty += quad.computePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna, totalGradHeading);
      /***********************************************/
      // penalty += quad.computeSimplePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/
      // penalty += quad.computeRobustSimplePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/
      // penalty += quad.computeRobustPenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/
      node = (j == 0 || j == numCheckPerPiece) ? 0.5 : 1.0;
      alpha = j * integralFrac;

      gradC.block<NUM_COEFF, PATH_DIM>(i * NUM_COEFF, 0) +=
          (beta.col(0) * totalGradPos.transpose() +
           beta.col(1) * totalGradVel.transpose() +
            beta.col(2) * totalGradAcc.transpose() +
            beta.col(3) * totalGradJer.transpose() +
            beta.col(4) * totalGradSna.transpose()) * step * node;

      gradT(i) += (totalGradPos.dot(vel) + totalGradVel.dot(acc) +
                    totalGradAcc.dot(jer) + totalGradJer.dot(sna) +
                    totalGradSna.dot(cra)) * step * node * alpha + node * integralFrac * penalty;

      cost += node * step * penalty;
    }
  }

  return cost;
}


void TrajSE3Solver::computeBeta(const double t, Eigen::Ref<Eigen::Matrix<double, NUM_COEFF, 6>> beta) {
  double s1, s2, s3, s4, s5, s6, s7;
  s1 = t;  // t
  s2 = s1 * s1;
  s3 = s2 * s1;
  s4 = s2 * s2;
  s5 = s4 * s1;
  s6 = s4 * s2;
  s7 = s4 * s3;
  beta.col(0) << 1.0, s1, s2, s3, s4, s5, s6, s7;
  beta.col(1) << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
  beta.col(2) << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
  beta.col(3) << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
  beta.col(4) << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
  beta.col(5) << 0.0, 0.0, 0.0, 0.0, 0.0, 120.0, 720.0 * s1, 2520.0 * s2;
}
  
}