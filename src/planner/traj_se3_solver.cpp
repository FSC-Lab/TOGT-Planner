#include "drolib/planner/traj_se3_solver.hpp"

namespace drolib {

bool TrajSE3Solver::solve(const PVAJ3D &initState,
            const PVAJ3D &endState, 
            const PVAJ1D &initYaw,
            const PVAJ1D &endYaw,
            const QuadManifold &quad,
            const TrajParams &tparams,
            const LbfgsParams& lbfgs) {
  // if (!data.valid()) {
  //   std::cout << "Data is not initialized properly.\n";
  //   return false;
  // }
  this->quad = quad; 
  this->tparams = tparams; 

  minco.setConditions(initState, endState, data.totalPieces);
  minco_yaw.setConditions(initYaw, endYaw, data.totalPieces);

  // minco_yaw.setParameters(data.Y, data.T);
  // std::cout << "coeffs: " << minco_yaw.getCoeffs() << std::endl;
  // std::cout << "initYaw: " << initYaw << std::endl;
  // std::cout << "endYaw: " << endYaw << std::endl;

  double cost{0.0};
  int ret = lbfgs_optimize(data.x, cost, &TrajSE3Solver::costFunction, nullptr, nullptr, this, lbfgs.params);

  if (ret < 0) {
    status = Status::LBFGS_FAILED;
    return false;
  }

  int total_dim = data.temporalVarDim + data.spatialVarDim + data.headingVarDim;


  Eigen::Map<const Eigen::VectorXd> K(data.x.data(), data.temporalVarDim);
  Eigen::Map<const Eigen::VectorXd> D(data.x.data() + data.temporalVarDim, data.spatialVarDim);
  Eigen::Map<const Eigen::VectorXd> Z(data.x.data() + data.temporalVarDim + data.spatialVarDim, data.headingVarDim);

  TrajSE3Data::forwardT(K, data.T);
  TrajSE3Data::forwardP(D, data.waypoints, data.P);
  TrajSE3Data::forwardY(Z, data.Y);


  status = static_cast<Status>(ret);
  minco.getTrajectory(data.traj_xyz);
  minco_yaw.getTrajectory(data.traj_yaw);


  return true;
}

double TrajSE3Solver::costFunction(void *ptr, const Eigen::VectorXd &x,
                                  Eigen::VectorXd &g) {
  // std::cout << "---------------------------" << std::endl;
  TrajSE3Solver *obj = reinterpret_cast<TrajSE3Solver *>(ptr);
  const int dimK = obj->data.temporalVarDim;
  const int dimD = obj->data.spatialVarDim;
  const int dimY = obj->data.headingVarDim;
  // Obtain current variables with gradients
  Eigen::Map<const Eigen::VectorXd> K(x.data(), dimK);
  Eigen::Map<const Eigen::VectorXd> D(x.data() + dimK, dimD);
  Eigen::Map<const Eigen::VectorXd> Z(x.data() + dimK + dimD, dimY);
  // std::cout << "Z: " << Z.transpose() << std::endl;

  Eigen::Map<Eigen::VectorXd> gradK(g.data(), dimK);
  Eigen::Map<Eigen::VectorXd> gradD(g.data() + dimK, dimD);
  Eigen::Map<Eigen::VectorXd> gradZ(g.data() + dimK + dimD, dimY);

  TrajSE3Data::forwardT(K, obj->data.T);
  TrajSE3Data::forwardP(D, obj->data.waypoints, obj->data.P);
  TrajSE3Data::forwardY(Z, obj->data.Y);
  // std::cout << "Y: " << obj->data.Y.transpose() << std::endl;
  std::cout << "x: " << x.transpose() << std::endl;
  std::cout << "Z: " << Z.transpose() << std::endl;
  std::cout << "Y: " << obj->data.Y.transpose() * 180.0 / M_PI << std::endl;

  double cost{0.0};
  obj->data.partialGradByTimes.setZero();
  obj->data.partialGradByCoeffs.setZero();
  obj->data.partialGradByHeadingCoeffs.setZero();

  obj->minco.setParameters(obj->data.P, obj->data.T);
  obj->minco_yaw.setParameters(obj->data.Y, obj->data.T);


  // cost += addEnergyCost(obj->minco, obj->minco_yaw, obj->tparams, obj->data.partialGradByTimes, obj->data.partialGradByCoeffs, obj->data.partialGradByHeadingCoeffs);
  cost += addPenaltyCost(obj->data.T, obj->minco.getCoeffs(), obj->minco_yaw.getCoeffs(), obj->quad, obj->yawTilt.get(), obj->tparams, obj->data.partialGradByTimes, obj->data.partialGradByCoeffs, obj->data.partialGradByHeadingCoeffs);
  std::cout << "obj->data.partialGradByHeadingCoeffs: " << obj->data.partialGradByHeadingCoeffs.norm() << std::endl;
  obj->minco.propagateGrad(obj->data.partialGradByCoeffs, obj->data.partialGradByTimes, obj->data.gradByPoints, obj->data.gradByXYZTimes);
  obj->minco_yaw.propagateGrad(obj->data.partialGradByHeadingCoeffs, obj->data.partialGradByTimes, obj->data.gradByYaw, obj->data.gradByYawTimes);
  
  std::cout << "gradByYaw: " << obj->data.gradByYaw.norm() << std::endl;
  std::cout << "gradByYawTimes: " << obj->data.gradByYawTimes.norm() << std::endl;
  // obj->data.gradByYawTimes.setZero();
  obj->data.gradByTimes = obj->data.gradByYawTimes + obj->data.gradByXYZTimes;

  cost += addTimeCost(obj->data.T, obj->tparams, obj->data.gradByTimes);

  TrajSE3Data::backPropagateT(K, obj->data.gradByTimes, gradK);
  TrajSE3Data::backPropagateP(D, obj->data.gradByPoints, obj->data.waypoints, gradD);
  TrajSE3Data::backPropagateY(Z, obj->data.gradByYaw, gradZ);

  // std::cout << "gradZ: " << gradZ.transpose() << std::endl;
  std::cout << "gradZ: " << gradZ.norm() << std::endl;


  return cost;
}

double TrajSE3Solver::addEnergyCost(const MincoSnap &minco_path, const MincoSnap1D &minco_heading,
                                    const TrajParams &params,
                                    Eigen::VectorXd &gradT,
                                    Eigen::MatrixX3d &gradC,
                                    Eigen::VectorXd &gradHeadingC) {
  double cost{0.0};
  if (params.weightEnergy <= 1.0e-6) {
    return cost;
  } 
  double cost_path{0.0};   
  Eigen::VectorXd gradT_path;                      
  minco_path.getEnergyWithGrads(cost_path, gradC, gradT_path);

  double cost_heading{0.0};                          
  Eigen::VectorXd gradT_heading;                      
  minco_heading.getEnergyWithGrads(cost_heading, gradHeadingC, gradT_heading);
  cost = cost_path * params.weightEnergy + cost_heading * params.weightEnergy;
  gradT = gradT_path * params.weightEnergy + gradT_heading * params.weightEnergy;
  gradC *= params.weightEnergy;
  gradHeadingC *= params.weightEnergy;
  return cost;
}

double TrajSE3Solver::addTimeCost(const Eigen::VectorXd &T,
                                  const TrajParams &params,
                                  Eigen::VectorXd &gradByTimes) {
  gradByTimes.array() += params.weightTime;
  return params.weightTime * T.sum();
}


double TrajSE3Solver::addPenaltyCost(const Eigen::VectorXd &T,
                                    const Eigen::MatrixX3d &path_coeffs,
                                    const Eigen::VectorXd &heading_coeffs,
                                    const QuadManifold &quad,
                                    AngleBase* yawTilt,
                                    const TrajParams &params,   
                                    Eigen::VectorXd &gradT,    
                                    Eigen::MatrixX3d &gradC,
                                    Eigen::VectorXd &gradHeadingC) {
  double cost{0.0};
  double step, alpha;
  double node, penalty;
  Eigen::Matrix<double, NUM_COEFF, 6> beta;
  Eigen::Vector3d pos, vel, acc, jer, sna, cra;
  double yaw, yawrate, yawacc, yawjer;
  Eigen::Vector3d heading;
  PVAJS3D pvajs;
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
    const Eigen::Matrix<double, NUM_COEFF, PATH_DIM> &path_c = path_coeffs.block<NUM_COEFF, PATH_DIM>(i * NUM_COEFF, 0);
    Eigen::Matrix<double, NUM_COEFF, YAW_DIM> heading_c = heading_coeffs.segment<NUM_COEFF>(i * NUM_COEFF);
    // heading_c.setZero();

    const double integralFrac = 1.0f / numCheckPerPiece;
    step = T(i) * integralFrac;
    for (int j = 0; j <= numCheckPerPiece; j++) {
      // std::cout << "==============dt: " <<  j * step << std::endl;

      computeBeta(j * step, beta);
      pos = path_c.transpose() * beta.col(0);
      vel = path_c.transpose() * beta.col(1);
      acc = path_c.transpose() * beta.col(2);
      jer = path_c.transpose() * beta.col(3);
      sna = path_c.transpose() * beta.col(4);
      cra = path_c.transpose() * beta.col(5);

      
      //TODO: compute yaw angle from another polynomial
      // yaw = yawTilt->at(j * step);
      yaw = (heading_coeffs.transpose() * beta.col(0));
      yawrate = (heading_coeffs.transpose() * beta.col(1));
      yawacc = (heading_coeffs.transpose() * beta.col(2));
      yawjer = (heading_coeffs.transpose() * beta.col(3));
       
      heading << yaw, yawrate, yawacc;
      // heading.setZero();
      // std::cout << "heading_coeffs: " << heading_coeffs << std::endl;
      std::cout << "heading: " << heading.transpose() << std::endl;

      pvajs << pos, vel, acc, jer, sna;
      penalty = 0.0;

      /***********************************************/
      // penalty += quad.computePenalityCost(pvajs, yaw, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna);
      // penalty += quad.computePenalityCost(pvajs, heading, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna, totalGradHeading);
      penalty += quad.computePenalityCostAD(pvajs, heading, params, totalGradPos, totalGradVel, totalGradAcc, totalGradJer, totalGradSna, totalGradHeading);
      // std::cout << "totalGradHeading: " << totalGradHeading << std::endl;
      // totalGradHeading.setZero();
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

      gradHeadingC.block<NUM_COEFF, YAW_DIM>(i * NUM_COEFF, 0) +=
          (beta.col(0) * totalGradHeading(0) +
           beta.col(1) * totalGradHeading(1) +
            beta.col(2) * totalGradHeading(2)) * step * node;

      std::cout << "gradHeadingC: " << gradHeadingC.norm() << std::endl;
      gradT(i) += (totalGradPos.dot(vel) + totalGradVel.dot(acc) +
                    totalGradAcc.dot(jer) + totalGradJer.dot(sna) +
                    totalGradSna.dot(cra) 
                    + totalGradHeading(0) * yawrate 
                    + totalGradHeading(1) * yawacc
                    + totalGradHeading(2) * yawjer) * step * node * alpha + node * integralFrac * penalty;

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