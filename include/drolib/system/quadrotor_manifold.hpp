#pragma once

#include <Eigen/Eigen>
#include "drolib/system/quadrotor_params.hpp"
#include "drolib/type/quad_state.hpp"
#include "drolib/type/command.hpp"
#include "drolib/type/set_point.hpp"
#include "drolib/math/gravity.hpp"
#include "drolib/planner/traj_params.hpp"

#include <cmath>

namespace drolib {

class QuadManifold {
 public:
  QuadManifold();
  QuadManifold(const QuadParams params);
  ~QuadManifold();

  bool toStateWithTiltYaw(const double t, const PVAJS &input, const Eigen::Vector3d& yaw, Setpoint &output) const;
  mutable Eigen::Quaterniond q_tilt_last_{1, 0, 0, 0};

  bool toStateWithTrueYaw(const double t, const PVAJS &input, const Eigen::Vector3d& yaw, Setpoint &output) const;

  bool computeThrustBodyrates(const PVAJS &input, const Eigen::Vector3d& yaw, double& thrust, double& bodyrateXY, double& bodyrateZ) const;

  void backPropagate(
      const Eigen::Vector3d &gradPos, const Eigen::Vector3d &gradVel,
      const Eigen::Vector4d &gradQuat, const Eigen::Vector3d &gradOmg,
      const Eigen::Vector4d &gradThr, Eigen::Vector3d &gradTotalPos,
      Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
      Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;

  void backPropagateSimple(
      const Eigen::Vector4d &gradQuat,
      const Eigen::Vector3d &gradOmg,
      const double gradThr, 
      Eigen::Vector3d &gradTotalAcc,
      Eigen::Vector3d &gradTotalJer) const;

/****************************************************/
  double computePenalityCost(
      const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos,
      Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
      Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;

  double computeSimplePenalityCost(
      const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos,
      Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
      Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;

//   double computeFastRacingPenalityCost(
//       const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
//       Eigen::Vector3d &gradTotalPos,
//       Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
//       Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;

  double computeRobustPenalityCost(
      const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos,
      Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
      Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;

  double computeRobustSimplePenalityCost(
      const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos,
      Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
      Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;



  double addThrustPenality(const double thrust, 
                                    const TrajParams &params,
                                    double& gradThrust) const;

  double addThrustsPenalities(const Eigen::Vector4d &thrusts, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector4d> gradThrusts) const;

  double addVelocityPenalities(const Eigen::Vector3d &vel, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector3d> gradVel) const;

  double addBodyratePenalities(const Eigen::Vector3d &omg, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector3d> gradOmg) const;

  double addRotationPenalities(const Eigen::Vector4d &quat, 
                                    const TrajParams &params,
                                    Eigen::Ref<Eigen::Vector4d> gradQuat) const;

  double addBoundaryPenalities(const Eigen::Vector3d &pos, 
                                      const TrajParams &params,
                                      Eigen::Ref<Eigen::Vector3d> gradPos) const;   

  bool smoothedL1(const double &x, const double &mu, double &f, double &df) const;
/****************************************************/

  void calcJacobian(void) const;

  inline bool valid(void) const { return params_.valid(); }

 private:
  QuadParams params_;

  Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();  // 3x3 identity
  
  double inertiaGapZY{};
  double inertiaGapXZ{};
  double inertiaGapYX{};
  Eigen::Matrix<double, 4, 3> matTorque;
  Eigen::Matrix<double, 4, 1> matThrust;

  // Cache variables
  mutable double a0, a1, a2, j0, j1, j2, s0, s1, s2;
  mutable double tilt_den, tilt_den_2, tilt_den_3, tilt0, tilt1, tilt2, c_half_psi,
      s_half_psi;
  mutable double c_psi, s_psi, omg_den, omg_den_2, omg_den_3, omg_den_4, omg_term;
  mutable double ng00, ng01, ng02, ng11, ng12, ng22;

  mutable double alpha0, alpha1, alpha2;
  mutable double zB0, zB1, zB2;
  mutable double dzB0, dzB1, dzB2;
  mutable double ddzB0, ddzB1, ddzB2;
  mutable double alpha_sqr0, alpha_sqr1, alpha_sqr2;
  mutable double alpha01, alpha12, alpha02;

  mutable double alpha_norm_1{}, alpha_norm_2{}, alpha_norm_3{}, alpha_norm_5{},
      alpha_norm_7{};
  mutable double alpha_dot_j{}, alpha_dot_s{};
  mutable double alpha_dot_j_sqr{};
  mutable double j_norm_2{};
  mutable double zB2_1{};
  
  mutable double tmp_omg_1{};
  mutable double tmp_omg_2{};
  mutable double tmp_omg_3{};
  mutable double tmp_omg_4{};
  mutable double tmp_omg_5{};
  mutable double tmp_omg_6{};
  mutable double tmp_quat_1{};
  mutable double tmp_quat_2{};

  mutable Eigen::Vector3d alpha;
  mutable Eigen::Vector3d zB, dzB, ddzB;
  mutable Eigen::Matrix3d DN_alpha;
  mutable Eigen::Vector3d DN_alpha_s;
  mutable Eigen::Matrix3d mat_DNalphas_a;
  mutable Eigen::Matrix3d mat_zB_a, mat_dzB_a, mat_ddzB_a;
  mutable Eigen::Matrix3d mat_dzB_j, mat_ddzB_j;
  mutable Eigen::Matrix3d mat_ddzB_s;
  mutable double dzB2_sqr{};

  mutable Eigen::Vector3d omg_dot;

  mutable Eigen::Matrix3d mat_w_zB;
  mutable Eigen::Matrix3d mat_w_dzB;
  mutable Eigen::Matrix3d mat_dw_zB;
  mutable Eigen::Matrix3d mat_dw_dzB;
  mutable Eigen::Matrix3d mat_dw_ddzB;

  mutable Eigen::Matrix3d mat_tor_w;
  mutable Eigen::Matrix3d mat_tor_dw;
  mutable Eigen::Vector3d gradTorque;
  mutable double gradCollectiveThr{};

  mutable Eigen::Vector3d d_Cf_a;
  mutable Eigen::Vector3d d_Cf_j;
  mutable Eigen::Vector3d d_Cf_s;
  mutable Eigen::Vector3d d_Cw_a;
  mutable Eigen::Vector3d d_Cw_j;
  mutable Eigen::Vector3d d_Cq_a;
  mutable Eigen::Vector3d d_Cq_zB;
  double eps;
  
//Temporay
mutable double omg0;
mutable double omg1;
mutable double omg2;
mutable Eigen::Vector4d quat_tmp;
mutable Eigen::Vector3d omg_tmp;
mutable Eigen::Vector3d tau_tmp;
mutable Eigen::Vector4d thrusts_tmp;
};

}