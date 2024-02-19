#pragma once

#include <Eigen/Eigen>
#include "drolib/planner/traj_data.hpp"
#include "drolib/planner/traj_params.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"
#include "drolib/solver/lbfgs_params.hpp"
#include "drolib/solver/minco_snap.hpp"
#include "drolib/solver/lbfgs.hpp"
#include "drolib/system/quadrotor_manifold.hpp"
#include "drolib/type/types.hpp"
#include <deque>
#include "drolib/planner/angle.hpp"

namespace drolib {

class TrajSolver {
 public:
  enum class Status : int {
    LBFGS_CONVERGENCE = 0,
    LBFGS_STOP,
    LBFGS_CANCELED,
    LBFGSERR_MAXIMUMITERATION,
    LBFGS_FAILED = -1
  };

  TrajSolver() = default;
  ~TrajSolver() {}

  bool setInitialGuess(const TrajData &tdata);

  void setConstYawBeforeTilt(const double& yaw = 0.0) {
    yawTilt = std::make_shared<ConstAngle>(yaw);
  }
  
  bool solve(const PVAJ &initState, const PVAJ &endState, const QuadManifold &quad,
             const TrajParams &tparams, const LbfgsParams &lbfgs);

  bool getTraejctory(PiecewisePolynomial<POLY_DEG> &trajectory) const {
    if (status == Status::LBFGS_FAILED) {
      return false;
    }
    trajectory = data.traj;
    return true;
  }
  // bool getWaypoints() const;

 public:
  MincoSnap minco;
  Status status{Status::LBFGS_FAILED};
  TrajData data;

  QuadManifold quad;
  TrajParams tparams;
  std::shared_ptr<AngleBase> yawTilt;
  
  static double costFunction(void *ptr, const Eigen::VectorXd &x,
                             Eigen::VectorXd &g);

  static double addEnergyCost(const MincoSnap &minco, const TrajParams &params,
                              Eigen::MatrixX3d &gradC, Eigen::VectorXd &gradT);

  static double addTimeCost(const Eigen::VectorXd &T, const TrajParams &params,
                            Eigen::VectorXd &gradByTimes);

  static double addPenaltyCost(const Eigen::VectorXd &T,
                               const Eigen::MatrixX3d &coeffs,
                               const QuadManifold &quad,
                               AngleBase* yawTilt,
                               const TrajParams &params,  
                               Eigen::MatrixX3d &gradC, Eigen::VectorXd &gradT);

  static double addRobustPenaltyCost(const Eigen::VectorXd &T,
                                     const Eigen::MatrixX3d &coeffs,
                                     const QuadManifold &quad,
                                     AngleBase* yawTilt,
                                     const TrajParams &params,  
                                     Eigen::MatrixX3d &gradC, Eigen::VectorXd &gradT);

  static double addThrustPenality(const double thrust, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Matrix<double, 1, 1>> gradThrust);

  static double addThrustsPenalities(const Eigen::Vector4d &thrusts, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector4d> gradThrusts);

  static double addVelocityPenalities(const Eigen::Vector3d &vel, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector3d> gradVel);

  static double addBodyratePenalities(const Eigen::Vector3d &omg, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector3d> gradOmg);

  // static double addBodyrateNormPenality(const double bodyrateNormSqrXY, const double bodyrateNormSqrZ,
  //                                   const TrajParams &params,
  //                                   Eigen::Ref<Eigen::Vector3d> gradOmg);

  static double addRotationPenalities(const Eigen::Vector4d &quat, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector4d> gradQuat);

  static double addBoundaryPenalities(const Eigen::Vector3d &pos, 
                                      const TrajParams &params,
                                      Eigen::Ref<Eigen::Vector3d> gradPos);                               

  static void computeBeta(const double t, Eigen::Ref<Eigen::Matrix<double, NUM_COEFF, 6>> beta);

  static void forwardT(const Eigen::VectorXd &K, Eigen::VectorXd &T);

  static void backwardT(const Eigen::VectorXd &T,
                        Eigen::Map<Eigen::VectorXd> &K);

  static void backPropagateT(const Eigen::VectorXd &K,
                             const Eigen::VectorXd &gradT,
                             Eigen::Map<Eigen::VectorXd> &gradK);

  static void forwardP(const Eigen::VectorXd &D,
                       const std::deque<Waypoint> &waypoints,
                       Eigen::Matrix3Xd &P);

  static void backwardP(const Eigen::Matrix3Xd &P,
                        const std::deque<Waypoint> &waypoints,
                        Eigen::Map<Eigen::VectorXd> &D);

  static void backPropagateP(const Eigen::VectorXd &D,
                             const Eigen::Matrix3Xd &gradP,
                             const std::deque<Waypoint> &waypoints,
                             Eigen::Map<Eigen::VectorXd> &gradD);

  static bool smoothedL1(const double &x, const double &mu, double &f,
                         double &df);
};

}  // namespace drolib