#ifndef DROLIB_PLANNER_TRAJ_SE3_SOLVER_HPP_
#define DROLIB_PLANNER_TRAJ_SE3_SOLVER_HPP_


#include <Eigen/Eigen>
#include "drolib/planner/traj_se3_data.hpp"
#include "drolib/planner/traj_se3_params.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"
#include "drolib/solver/lbfgs_params.hpp"
#include "drolib/solver/minco_snap.hpp"

#include "drolib/solver/lbfgs.hpp"
#include "drolib/system/quadrotor_manifold.hpp"
#include "drolib/type/types.hpp"
#include <deque>
#include "drolib/planner/angle.hpp"

namespace drolib {

class TrajSE3Solver {
 public:
  enum class Status : int {
    LBFGS_CONVERGENCE = 0,
    LBFGS_STOP,
    LBFGS_CANCELED,
    LBFGSERR_MAXIMUMITERATION,
    LBFGS_FAILED = -1
  };

  TrajSE3Solver() = default;
  ~TrajSE3Solver() {}

  static void computeBeta(const double t, Eigen::Ref<Eigen::Matrix<double, NUM_COEFF, 6>> beta);

  bool solve(const PVAJ &initState, const PVAJ &endState, const Eigen::Vector3d &initYaw,
            const Eigen::Vector3d &endYaw, const QuadManifold &quad,
             const TrajParams &tparams, const LbfgsParams &lbfgs);

  // bool getTraejctory(PiecewisePolynomial<POLY_DEG> &trajectory) const {
  //   if (status == Status::LBFGS_FAILED) {
  //     return false;
  //   }
  //   trajectory = data.traj;
  //   return true;
  // }
  // // bool getWaypoints() const;

 public:
  MincoSnap minco;
  MincoSnap1D minco_yaw;
  Status status{Status::LBFGS_FAILED};
  TrajSE3Data data;

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
};

}

#endif  /* DROLIB_PLANNER_TRAJ_SE3_SOLVER_HPP_ */
