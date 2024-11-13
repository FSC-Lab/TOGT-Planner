#include "drolib/planner/traj_solver.hpp"

namespace drolib {

bool TrajSolver::setInitialGuess(const TrajData &tdata) {
  if (!tdata.valid()) {
    return false;
  }
  data = tdata;
  return true;
}

bool TrajSolver::solve(const PVAJ &initState,
            const PVAJ &endState, 
            const QuadManifold &quad,
            const TrajParams &tparams,
            const LbfgsParams& lbfgs) {
  if (!data.valid()) {
    std::cout << "Data is not initialized properly.\n";
    return false;
  }

  if (!yawTilt) {
    std::cout << "Yaw angle not given.\n";
    return false;
  }

  this->quad = quad; 
  this->tparams = tparams; 

  minco.setConditions(initState, endState, data.totalPieces);

  double cost{0.0};
  int ret = lbfgs_optimize(data.x, cost, &TrajSolver::costFunction, nullptr, nullptr, this, lbfgs.params);

  // std::cout << "minimum cost: " << cost << "\n";
  if (ret < 0) {
    status = Status::LBFGS_FAILED;
    return false;
  }

  int total_dim = data.temporalVarDim + data.spatialVarDim;
  // std::cout << "total_dim: " << total_dim << std::endl;

  Eigen::Map<const Eigen::VectorXd> K(data.x.data(), data.temporalVarDim);
  Eigen::Map<const Eigen::VectorXd> D(data.x.data() + data.temporalVarDim, data.spatialVarDim);
  forwardT(K, data.T);
  forwardP(D, data.waypoints, data.P);

  status = static_cast<Status>(ret);
  minco.getTrajectory(data.traj);


  return true;
}

double TrajSolver::costFunction(void *ptr, const Eigen::VectorXd &x,
                                  Eigen::VectorXd &g) {
  TrajSolver *obj = reinterpret_cast<TrajSolver *>(ptr);
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

double TrajSolver::addEnergyCost(const MincoSnap &minco,
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

double TrajSolver::addTimeCost(const Eigen::VectorXd &T,
                                  const TrajParams &params,
                                  Eigen::VectorXd &gradByTimes) {
  gradByTimes.array() += params.weightTime;
  return params.weightTime * T.sum();
}

double TrajSolver::addRobustPenaltyCost(const Eigen::VectorXd &T,
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
  Eigen::Vector3d gradPos;
  Eigen::Vector3d gradVel;
  Eigen::Vector3d gradOmg;
  Eigen::Vector4d gradQuat;
  Eigen::Matrix<double, 1, 1> gradThrust;
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

      //TODO: set up yaw correctly
      yaw = yawTilt->at(j * step);

      pvajs << pos, vel, acc, jer, sna;
      double thrust, bodyrateXY, bodyrateZ;
      quad.computeThrustBodyrates(pvajs, yaw, thrust, bodyrateXY, bodyrateZ);

      // if(!quad.toStateWithTiltYaw(0.0, pvajs, yaw, setpoint)){
      //   continue;
      // }

      penalty = 0.0;
      penalty += addVelocityPenalities(vel, params, gradVel);
      penalty += addThrustPenality(thrust, params, gradThrust);
      // penalty += addRotationPenalities(setpoint.state.qx, params, gradQuat);
      // penalty += addBodyratePenalities(setpoint.state.w, params, gradOmg);
      // penalty += addThrustsPenalities(setpoint.input.thrusts, params, gradThrusts);
      // //TODO: not tested yet
      penalty += addBoundaryPenalities(setpoint.state.p, params, gradPos);

      node = (j == 0 || j == numCheckPerPiece) ? 0.5 : 1.0;
      alpha = j * integralFrac;

      quad.backPropagate(gradPos, gradVel, gradQuat, gradOmg,
                              gradThrusts, totalGradPos, totalGradVel,
                              totalGradAcc, totalGradJer, totalGradSna);

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

double TrajSolver::addPenaltyCost(const Eigen::VectorXd &T,
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

      //TODO: set up yaw correctly
      yaw = yawTilt->at(j * step);

      pvajs << pos, vel, acc, jer, sna;
      penalty = 0.0;

      /***********************************************/
      // penalty += quad.computePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/
      // penalty += quad.computeSimplePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/
      // penalty += quad.computeRobustSimplePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/
      penalty += quad.computeRobustPenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      /***********************************************/


      // if(!quad.toStateWithTiltYaw(0.0, pvajs, yaw, setpoint)){
      //   continue;
      // }

      // penalty = 0.0;
      // penalty += addVelocityPenalities(setpoint.state.v, params, gradVel);
      // penalty += addRotationPenalities(setpoint.state.qx, params, gradQuat);
      // penalty += addBodyratePenalities(setpoint.state.w, params, gradOmg);
      // penalty += addThrustsPenalities(setpoint.input.thrusts, params, gradThrusts);
      // //TODO: not tested yet
      // penalty += addBoundaryPenalities(setpoint.state.p, params, gradPos);

      // quad.backPropagate(gradPos, gradVel, gradQuat, gradOmg,
      //                         gradThrusts, totalGradPos, totalGradVel,
      //                         totalGradAcc, totalGradJer, totalGradSna);
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

double TrajSolver::addThrustPenality(const double thrust, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Matrix<double, 1, 1>> gradThrust) {
  double penalty{0.0};     
  gradThrust.setZero();
  if (params.weightThr <= 1.0e-6) {
    return penalty;
  }  

  double v, vPena, vPenaD;
  v = (thrust - params.collectivtThrMean) * (thrust - params.collectivtThrMean) - params.collectivtThrRadiSqr;
  if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
    gradThrust(0) += params.weightThr * vPenaD * 2.0 * (thrust - params.collectivtThrMean);
    penalty += params.weightThr * vPena;
  }

  return penalty;                       
}

double TrajSolver::addThrustsPenalities(const Eigen::Vector4d &thrusts, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector4d> gradThrusts) {
  double penalty{0.0};
  gradThrusts.setZero();

  if (params.weightThr <= 1.0e-6) {
    return penalty;
  }  

  double v, vPena, vPenaD;
  for (size_t i{0}; i < 4; ++i) {
    v = (thrusts(i) - params.thrMean) * (thrusts(i) - params.thrMean) - params.thrRadiSqr;
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradThrusts(i) += params.weightThr * vPenaD * 2.0 * (thrusts(i) - params.thrMean);
      penalty += params.weightThr * vPena;
    }
  }
  return penalty;
}

double TrajSolver::addVelocityPenalities(const Eigen::Vector3d &vel, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector3d> gradVel) {
  double penalty{0.0};
  gradVel.setZero();

  if (params.weightVel <= 1.0e-6) {
    return penalty;
  }                                 

  double v, vPena, vPenaD;
  v = vel.squaredNorm() - params.maxVelSqr;
  if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
    gradVel += params.weightVel * vPenaD * 2.0 * vel;
    penalty += params.weightVel * vPena;
  }
  return penalty;
}

// double TrajSolver::addBodyrateNormPenality(const double bodyrateNormSqrXY, 
//                                            const double bodyrateNormSqrZ,
//                                            const TrajParams &params,
//                                            Eigen::Ref<Eigen::Vector3d> gradOmg) {
//   double penalty{0.0};
//   gradOmg.setZero();

//   if (params.weightOmg <= 1.0e-6) {
//     return penalty;
//   }  

//   double vxy, vz, vPena, vPenaD;
//   vxy = bodyrateNormSqrXY - params.maxOmgXYSqr;
//   vz = bodyrateNormSqrZ - params.maxOmgZSqr;
//   if (smoothedL1(vxy, params.smoothingEps, vPena, vPenaD)) {
//     gradOmg.head<2>() += params.weightOmg * vPenaD * 2.0 * omg.head<2>();
//     penalty += params.weightOmg * vPena;
//   }
//   if (smoothedL1(vz, params.smoothingEps, vPena, vPenaD)) {
//     gradOmg.z() += params.weightOmg * vPenaD * 2.0 * omg.z();
//     penalty += params.weightOmg * vPena;
//   }

//   return penalty;
// }


double TrajSolver::addBodyratePenalities(const Eigen::Vector3d &omg, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector3d> gradOmg) {
  double penalty{0.0};
  gradOmg.setZero();

  if (params.weightOmg <= 1.0e-6) {
    return penalty;
  }  

  double vxy, vz, vPena, vPenaD;
  vxy = omg.head<2>().squaredNorm() - params.maxOmgXYSqr;
  vz = omg.z() * omg.z() - params.maxOmgZSqr;
  if (smoothedL1(vxy, params.smoothingEps, vPena, vPenaD)) {
    gradOmg.head<2>() += params.weightOmg * vPenaD * 2.0 * omg.head<2>();
    penalty += params.weightOmg * vPena;
  }
  if (smoothedL1(vz, params.smoothingEps, vPena, vPenaD)) {
    gradOmg.z() += params.weightOmg * vPenaD * 2.0 * omg.z();
    penalty += params.weightOmg * vPena;
  }

  return penalty;
}

double TrajSolver::addRotationPenalities(const Eigen::Vector4d &quat, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector4d> gradQuat) {
  double penalty{0.0};
  gradQuat.setZero();

  if (params.weightRot <= 1.0e-6) {
    return penalty;
  }  

  double v, vPena, vPenaD;
  const double cosAng = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
  v = std::acos(cosAng) - params.maxTiltedAngle;
  if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
    gradQuat += params.weightRot * vPenaD /  sqrt(1.0 - cosAng * cosAng) * 4.0 * Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
    penalty += params.weightRot * vPena;
  }

  return penalty;
}

double TrajSolver::addBoundaryPenalities(const Eigen::Vector3d &pos, 
                                      const TrajParams &params,
                                      Eigen::Ref<Eigen::Vector3d> gradPos) {
  double penalty{0.0};
  gradPos.setZero();

  if (params.weightPos <= 1.0e-6) {
    return penalty;
  }                                 

  double v, vPena, vPenaD;
  // Minimum boundary
  if (params.boundX(0) > -1.0e4) {
    v = params.boundX(0) - pos.x();
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradPos.x() += params.weightPos * vPenaD * (-1.0);
      penalty += params.weightPos * vPena;
    }
  }

  if (params.boundY(0) > -1.0e4) {
    v = params.boundY(0) - pos.y();
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradPos.y() += params.weightPos * vPenaD * (-1.0);
      penalty += params.weightPos * vPena;
    }
  }

  if (params.boundZ(0) > -1.0e4) {
    v = params.boundZ(0) - pos.z();
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradPos.z() += params.weightPos * vPenaD * (-1.0);
      penalty += params.weightPos * vPena;
    }
  }
  // Maximum boundary
  if (params.boundX(0) < 1.0e4) {
    v = pos.x() - params.boundX(1);
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradPos.x() += params.weightPos * vPenaD * (1.0);
      penalty += params.weightPos * vPena;
    }
  }

  if (params.boundY(0) < 1.0e4) {
    v = pos.y() - params.boundY(1);
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradPos.y() += params.weightPos * vPenaD * (1.0);
      penalty += params.weightPos * vPena;
    }
  }

  if (params.boundZ(0) < 1.0e4) {
    v = pos.z() - params.boundZ(1);
    if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
      gradPos.z() += params.weightPos * vPenaD * (1.0);
      penalty += params.weightPos * vPena;
    }
  }

  return penalty;
}

void TrajSolver::computeBeta(const double t, Eigen::Ref<Eigen::Matrix<double, NUM_COEFF, 6>> beta) {
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

void TrajSolver::forwardT(const Eigen::VectorXd &K, Eigen::VectorXd &T) {
  for (int i = 0; i < K.size(); i++) {
    T(i) = K(i) > 0.0 ? ((0.5 * K(i) + 1.0) * K(i) + 1.0)
                      : 1.0 / ((0.5 * K(i) - 1.0) * K(i) + 1.0);
    // T(i) = K(i) * K(i);
  }
  return;
}

void TrajSolver::backwardT(const Eigen::VectorXd &T,
                              Eigen::Map<Eigen::VectorXd> &K) {
  for (int i = 0; i < T.size(); i++) {
    K(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                      : (1.0 - sqrt(2.0 / T(i) - 1.0));

    // K(i) = sqrt(T(i));
  }

  return;
}

void TrajSolver::backPropagateT(const Eigen::VectorXd &K,
                                  const Eigen::VectorXd &gradT,
                                  Eigen::Map<Eigen::VectorXd> &gradK) {
  for (int i = 0; i < K.size(); i++) {
    if (K(i) > 0) {
      gradK(i) = gradT(i) * (K(i) + 1.0);
    } else {
      const double denSqrt = (0.5 * K(i) - 1.0) * K(i) + 1.0;
      gradK(i) = gradT(i) * (1.0 - K(i)) / (denSqrt * denSqrt);
    }

    // gradK(i) = gradT(i) * 2.0 * K(i); 
  }

  return;
}

void TrajSolver::forwardP(const Eigen::VectorXd &D,
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

void TrajSolver::backwardP(const Eigen::Matrix3Xd &P, const std::deque<Waypoint> &waypoints,
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

void TrajSolver::backPropagateP(const Eigen::VectorXd &D,
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

bool TrajSolver::smoothedL1(const double &x, const double &mu, double &f,
                              double &df) {
  if (x < 0.0) {
    return false;
  } else if (x > mu) {
    f = x - 0.5 * mu;
    df = 1.0;
    return true;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    f = mumxd2 * sqrxdmu * xdmu;
    df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return true;
  }
}
  
}