#pragma once

#include "drolib/math/gravity.hpp"
#include "drolib/planner/traj_params.hpp"
#include "drolib/system/quadrotor_params.hpp"
#include "drolib/type/command.hpp"
#include "drolib/type/quad_state.hpp"
#include "drolib/type/set_point.hpp"
#include <Eigen/Eigen>
#include <cmath>
#include <limits>

#include <casadi/casadi.hpp>
#include <casadi/core/casadi_misc.hpp>

namespace drolib {

class QuadManifold {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadManifold();
  QuadManifold(const QuadParams params);
  ~QuadManifold();

  bool toStateWithTiltYaw(const double t, const PVAJS3D &input,
                          const Eigen::Vector3d &yaw, Setpoint &output) const;

  bool toStateWithTiltYawMass(const double t, const PVAJS3D &input,
                              const Eigen::Vector3d &yaw,
                              Setpoint &output) const;

  bool toStateWithTiltYawAD(const double t, const PVAJS3D &input,
                            const Eigen::Vector3d &heading, Setpoint &output,
                            Eigen::Matrix<double, 11, 3> &jacVel,
                            Eigen::Matrix<double, 11, 3> &jacAcc,
                            Eigen::Matrix<double, 11, 3> &jacJer,
                            Eigen::Matrix<double, 11, 3> &jacSna,
                            Eigen::Matrix<double, 11, 3> &jacHeading) const;

  mutable Eigen::Quaterniond q_tilt_last_{1, 0, 0, 0};

  bool toStateWithTrueYaw(const double t, const PVAJS3D &input,
                          const Eigen::Vector3d &yaw, Setpoint &output) const;

  bool computeThrustBodyrates(const PVAJS3D &input, const Eigen::Vector3d &yaw,
                              double &thrust, double &bodyrateXY,
                              double &bodyrateZ) const;

  void
  backPropagate(const Eigen::Vector3d &gradPos, const Eigen::Vector3d &gradVel,
                const Eigen::Vector4d &gradQuat, const Eigen::Vector3d &gradOmg,
                const Eigen::Vector4d &gradThr, Eigen::Vector3d &gradTotalPos,
                Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
                Eigen::Vector3d &gradTotalJer,
                Eigen::Vector3d &gradTotalSna) const;

  void backPropagateSimple(const Eigen::Vector4d &gradQuat,
                           const Eigen::Vector3d &gradOmg, const double gradThr,
                           Eigen::Vector3d &gradTotalAcc,
                           Eigen::Vector3d &gradTotalJer) const;

  /****************************************************/
  double computePenalityCost(const PVAJS3D &pvajs, const Eigen::Vector3d &yaw,
                             const TrajParams &params,
                             Eigen::Vector3d &gradTotalPos,
                             Eigen::Vector3d &gradTotalVel,
                             Eigen::Vector3d &gradTotalAcc,
                             Eigen::Vector3d &gradTotalJer,
                             Eigen::Vector3d &gradTotalSna,
                             Eigen::Vector3d &gradTotalHeading) const;

  double computePenalityCostAD(const PVAJS3D &pvajs, const Eigen::Vector3d &yaw,
                             const TrajParams &params,
                             Eigen::Vector3d &gradTotalPos,
                             Eigen::Vector3d &gradTotalVel,
                             Eigen::Vector3d &gradTotalAcc,
                             Eigen::Vector3d &gradTotalJer,
                             Eigen::Vector3d &gradTotalSna,
                             Eigen::Vector3d &gradTotalHeading) const;


  double computeSimplePenalityCost(
      const PVAJS3D &pvajs, const Eigen::Vector3d &yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos, Eigen::Vector3d &gradTotalVel,
      Eigen::Vector3d &gradTotalAcc, Eigen::Vector3d &gradTotalJer,
      Eigen::Vector3d &gradTotalSna) const;

  //   double computeFastRacingPenalityCost(
  //       const PVAJS3D &pvajs, const Eigen::Vector3d& yaw, const TrajParams
  //       &params, Eigen::Vector3d &gradTotalPos, Eigen::Vector3d
  //       &gradTotalVel, Eigen::Vector3d &gradTotalAcc, Eigen::Vector3d
  //       &gradTotalJer, Eigen::Vector3d &gradTotalSna) const;

//   double computePenalityCostAD(
//       const PVAJS3D &pvajs, const Eigen::Vector3d &yaw, const TrajParams &params,
//       Eigen::Vector3d &gradTotalPos, Eigen::Vector3d &gradTotalVel,
//       Eigen::Vector3d &gradTotalAcc, Eigen::Vector3d &gradTotalJer,
//       Eigen::Vector3d &gradTotalSna, Eigen::Vector3d &gradToHeading) const;

  double computeRobustPenalityCost(
      const PVAJS3D &pvajs, const Eigen::Vector3d &yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos, Eigen::Vector3d &gradTotalVel,
      Eigen::Vector3d &gradTotalAcc, Eigen::Vector3d &gradTotalJer,
      Eigen::Vector3d &gradTotalSna) const;

  double computeRobustSimplePenalityCost(
      const PVAJS3D &pvajs, const Eigen::Vector3d &yaw, const TrajParams &params,
      Eigen::Vector3d &gradTotalPos, Eigen::Vector3d &gradTotalVel,
      Eigen::Vector3d &gradTotalAcc, Eigen::Vector3d &gradTotalJer,
      Eigen::Vector3d &gradTotalSna) const;

  double addThrustPenality(const double thrust, const TrajParams &params,
                           double &gradThrust) const;

  double addThrustsPenalities(const Eigen::Vector4d &thrusts,
                              const TrajParams &params,
                              Eigen::Ref<Eigen::Vector4d> gradThrusts) const;

  double addVelocityPenalities(const Eigen::Vector3d &vel,
                               const TrajParams &params,
                               Eigen::Ref<Eigen::Vector3d> gradVel) const;

  double addBodyratePenalities(const Eigen::Vector3d &omg,
                               const TrajParams &params,
                               Eigen::Ref<Eigen::Vector3d> gradOmg) const;

  double addPerceptionCost(const Eigen::Vector4d &quat,
                            const QuadParams &quad_params,
                            const TrajParams &params,
                            Eigen::Ref<Eigen::Vector4d> gradQuat) const;

  double addYawPenality(const Eigen::Vector4d &quat,
                            const TrajParams &params,
                            Eigen::Ref<Eigen::Vector4d> gradQuat) const;

  double addRotationPenalities(const Eigen::Vector4d &quat,
                               const TrajParams &params,
                               Eigen::Ref<Eigen::Vector4d> gradQuat) const;

  double addBoundaryPenalities(const Eigen::Vector3d &pos,
                               const TrajParams &params,
                               Eigen::Ref<Eigen::Vector3d> gradPos) const;

  bool smoothedL1(const double &x, const double &mu, double &f,
                  double &df) const;
  /****************************************************/

  void calcJacobian(void) const;

  inline bool valid(void) const { return quad_params_.valid(); }

private:
  QuadParams quad_params_;

  Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity(); // 3x3 identity

  double inertiaGapZY{};
  double inertiaGapXZ{};
  double inertiaGapYX{};
  Eigen::Matrix<double, 4, 3> matTorque;
  Eigen::Matrix<double, 4, 1> matThrust;

  // Cache variables
  mutable double a0, a1, a2, j0, j1, j2, s0, s1, s2;
  mutable double tilt_den, tilt_den_2, tilt_den_3, tilt0, tilt1, tilt2,
      c_half_psi, s_half_psi;
  mutable double c_psi, s_psi, omg_den, omg_den_2, omg_den_3, omg_den_4,
      omg_term;
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

  // Temporay
  mutable double omg0;
  mutable double omg1;
  mutable double omg2;
  mutable Eigen::Vector4d quat_tmp;
  mutable Eigen::Vector3d omg_tmp;
  mutable Eigen::Vector3d tau_tmp;
  mutable Eigen::Vector4d thrusts_tmp;

  casadi::Function fun_forward_singleThr_;
  casadi::Function fun_forward_singleThr_jacVel_;
  casadi::Function fun_forward_singleThr_jacAcc_;
  casadi::Function fun_forward_singleThr_jacJer_;
  casadi::Function fun_forward_singleThr_jacSna_;
  casadi::Function fun_forward_singleThr_jacHeading_;

  casadi::Function fun_perception_cost_;
  casadi::Function fun_perception_cost_jacQuat_;

  inline casadi::SX dot_SX(const casadi::SX &a, const casadi::SX &b) {
    return a(0) * b(0) + a(1) * b(1) + a(2) * b(2);
  }

  inline casadi::SX cross_SX(const casadi::SX &a, const casadi::SX &b) {
    return vertcat(-a(2) * b(1) + a(1) * b(2), a(2) * b(0) - a(0) * b(2),
                   -a(1) * b(0) + a(0) * b(1));
  }

  inline casadi::SX rotate_quat(const casadi::SX& q, const casadi::SX& v) {
    // Extract scalar (w) and vector (vec) parts from the quaternion
    // casadi::SX w = q(0);                    // Scalar part
    // casadi::SX vec = q(casadi::Slice(1, 4)); // Vector part

    // // Compute uv = 2 * cross(vec, v)
    // casadi::SX uv = cross_SX(vec, v);  
    // uv = 2 * uv;

    // // Return the rotated vector
    // return v + w * uv + cross_SX(vec, uv);

    casadi::SX q_conj = casadi::SX::vertcat({q(0), -q(1), -q(2), -q(3)});
    casadi::SX v_quat = casadi::SX::vertcat({0, v});
    casadi::SX ans = quat_mult(quat_mult(q, v_quat), q_conj);
    return casadi::SX::vertcat({ans(1), ans(2), ans(3)}); 


  }

  inline casadi::SX quat_mult(const casadi::SX& q1, const casadi::SX& q2) {
    // Quaternion multiplication
    casadi::SX ans = casadi::SX::vertcat({
        q2(0) * q1(0) - q2(1) * q1(1) - q2(2) * q1(2) - q2(3) * q1(3),
        q2(0) * q1(1) + q2(1) * q1(0) - q2(2) * q1(3) + q2(3) * q1(2),
        q2(0) * q1(2) + q2(2) * q1(0) + q2(1) * q1(3) - q2(3) * q1(1),
        q2(0) * q1(3) - q2(1) * q1(2) + q2(2) * q1(1) + q2(3) * q1(0)
    });
    return ans;
  }

  inline void initPerceptionCostCasadiFunc(const QuadParams params) {
    // casadi::DM gate_orient = vertcat(params.);
    double q_wg_w = params.gate_orient.w();
    double q_wg_x = params.gate_orient.x();
    double q_wg_y = params.gate_orient.y();
    double q_wg_z = params.gate_orient.z();

    double q_bc_w = params.q_bc.w();
    double q_bc_x = params.q_bc.x();
    double q_bc_y = params.q_bc.y();
    double q_bc_z = params.q_bc.z();  

    casadi::SX q_wb = casadi::SX::sym("q_wb", 4);


    // Create a CasADi SX representation of the quaternion
    casadi::SX q_wg = casadi::SX::vertcat({q_wg_w, q_wg_x, q_wg_y, q_wg_z});
    casadi::SX q_bc = casadi::SX::vertcat({q_bc_w, q_bc_x, q_bc_y, q_bc_z});
    casadi::SX q_wc = quat_mult(q_wb, q_bc);

    casadi::SX e_z = casadi::SX::vertcat({0.0, 0.0, 1.0});
    casadi::SX z_g = rotate_quat(q_wg, e_z);
    casadi::SX z_c = rotate_quat(q_wc, e_z);

    casadi::SX cost = 1 * (1.0 - dot_SX(z_c, z_g));

    fun_perception_cost_ =
        casadi::Function("perception_cost", {q_wb}, {cost});

    casadi::SX jac_q = jacobian(cost, q_wb);
    fun_perception_cost_jacQuat_ =
        casadi::Function("perceptoin_cost_jacQuat", {q_wb}, {densify(jac_q)});

  }


  inline void initSingleThrCasadiFunc(const QuadParams params) {
    const double I_x = params.inertia.x();
    const double I_y = params.inertia.y();
    const double I_z = params.inertia.z();
    const double grav = G;
    const double mass = params.mass;

    // function inuputs
    casadi::SX v_0 = casadi::SX::sym("v_0", 1);
    casadi::SX v_1 = casadi::SX::sym("v_1", 1);
    casadi::SX v_2 = casadi::SX::sym("v_2", 1);
    casadi::SX vel = vertcat(v_0, v_1, v_2);

    casadi::SX a_0 = casadi::SX::sym("a_0", 1);
    casadi::SX a_1 = casadi::SX::sym("a_1", 1);
    casadi::SX a_2 = casadi::SX::sym("a_2", 1);
    casadi::SX acc = vertcat(a_0, a_1, a_2);

    casadi::SX j_0 = casadi::SX::sym("j_0", 1);
    casadi::SX j_1 = casadi::SX::sym("j_1", 1);
    casadi::SX j_2 = casadi::SX::sym("j_2", 1);
    casadi::SX jer = vertcat(j_0, j_1, j_2);

    casadi::SX s_0 = casadi::SX::sym("s_0", 1);
    casadi::SX s_1 = casadi::SX::sym("s_1", 1);
    casadi::SX s_2 = casadi::SX::sym("s_2", 1);
    casadi::SX sna = vertcat(s_0, s_1, s_2);

    casadi::SX yaw = casadi::SX::sym("yaw", 1);
    casadi::SX dyaw = casadi::SX::sym("dyaw", 1);
    casadi::SX ddyaw = casadi::SX::sym("ddyaw", 1);
    casadi::SX heading = vertcat(yaw, dyaw, ddyaw);

    // alpha
    casadi::SX zu0_sx = a_0;
    casadi::SX zu1_sx = a_1;
    casadi::SX zu2_sx = a_2 + grav;

    casadi::SX zu_sqr0_sx = zu0_sx * zu0_sx;
    casadi::SX zu_sqr1_sx = zu1_sx * zu1_sx;
    casadi::SX zu_sqr2_sx = zu2_sx * zu2_sx;
    casadi::SX zu01_sx = zu0_sx * zu1_sx;
    casadi::SX zu12_sx = zu1_sx * zu2_sx;
    casadi::SX zu02_sx = zu0_sx * zu2_sx;

    // alpha norm
    casadi::SX zu_sqr_norm_sx = zu_sqr0_sx + zu_sqr1_sx + zu_sqr2_sx;
    casadi::SX zu_norm_sx = sqrt(zu_sqr_norm_sx);
    // alpha direction
    casadi::SX z0_sx = zu0_sx / zu_norm_sx;
    casadi::SX z1_sx = zu1_sx / zu_norm_sx;
    casadi::SX z2_sx = zu2_sx / zu_norm_sx;
    // alpha norm^3
    casadi::SX ng_den_sx = zu_sqr_norm_sx * zu_norm_sx;

    // some directions
    casadi::SX ng00_sx = (zu_sqr1_sx + zu_sqr2_sx) / ng_den_sx;
    casadi::SX ng01_sx = -zu01_sx / ng_den_sx;
    casadi::SX ng02_sx = -zu02_sx / ng_den_sx;
    casadi::SX ng11_sx = (zu_sqr0_sx + zu_sqr2_sx) / ng_den_sx;
    casadi::SX ng12_sx = -zu12_sx / ng_den_sx;
    casadi::SX ng22_sx = (zu_sqr0_sx + zu_sqr1_sx) / ng_den_sx;

    // vT*a
    casadi::SX v_dot_a_sx = v_0 * a_0 + v_1 * a_1 + v_2 * a_2;

    casadi::SX dz0_sx = ng00_sx * j_0 + ng01_sx * j_1 + ng02_sx * j_2;
    casadi::SX dz1_sx = ng01_sx * j_0 + ng11_sx * j_1 + ng12_sx * j_2;
    casadi::SX dz2_sx = ng02_sx * j_0 + ng12_sx * j_1 + ng22_sx * j_2;

    casadi::SX tilt_den_sx = sqrt(2.0 * (1.0 + z2_sx));
    casadi::SX tilt0_sx = 0.5 * tilt_den_sx;
    casadi::SX tilt1_sx = -z1_sx / tilt_den_sx;
    casadi::SX tilt2_sx = z0_sx / tilt_den_sx;
    casadi::SX c_half_yaw_sx = cos(0.5 * yaw);
    casadi::SX s_half_yaw_sx = sin(0.5 * yaw);
    casadi::SX c_yaw_sx = cos(yaw);
    casadi::SX s_yaw_sx = sin(yaw);
    casadi::SX omg_den_sx = z2_sx + 1.0;
    casadi::SX omg_term_sx = dz2_sx / omg_den_sx;

    // Rotation
    casadi::SX quat_w = tilt0_sx * c_half_yaw_sx;
    casadi::SX quat_x = tilt1_sx * c_half_yaw_sx + tilt2_sx * s_half_yaw_sx;
    casadi::SX quat_y = tilt2_sx * c_half_yaw_sx - tilt1_sx * s_half_yaw_sx;
    casadi::SX quat_z = tilt0_sx * s_half_yaw_sx;

    casadi::SX R00 =
        quat_w * quat_w + quat_x * quat_x - quat_y * quat_y - quat_z * quat_z;
    casadi::SX R10 = 2.0 * quat_w * quat_z + 2.0 * quat_x * quat_y;
    casadi::SX R20 = 2.0 * quat_x * quat_z - 2.0 * quat_w * quat_y;

    casadi::SX R01 = 2.0 * quat_x * quat_y - 2.0 * quat_w * quat_z;
    casadi::SX R11 =
        quat_w * quat_w - quat_x * quat_x + quat_y * quat_y - quat_z * quat_z;
    casadi::SX R21 = 2.0 * quat_w * quat_x + 2.0 * quat_y * quat_z;

    casadi::SX R02 = 2.0 * quat_w * quat_y + 2.0 * quat_x * quat_z;
    casadi::SX R12 = 2.0 * quat_y * quat_z - 2.0 * quat_w * quat_x;
    casadi::SX R22 =
        quat_w * quat_w - quat_x * quat_x - quat_y * quat_y + quat_z * quat_z;

    casadi::SX x_B = vertcat(R00, R10, R20);
    casadi::SX y_B = vertcat(R01, R11, R21);
    casadi::SX z_B = vertcat(R02, R12, R22);

    casadi::SX x_C = vertcat(c_yaw_sx, s_yaw_sx, 0.0);
    casadi::SX y_C = vertcat(-s_yaw_sx, c_yaw_sx, 0.0);

    // thrust
    casadi::SX thrust =
        z0_sx * mass * a_0 + z1_sx * mass * a_1 + z2_sx * (mass * (a_2 + grav));

    // Bodyrates
    casadi::SX omg_x = dz0_sx * s_yaw_sx - dz1_sx * c_yaw_sx -
                       (z0_sx * s_yaw_sx - z1_sx * c_yaw_sx) * omg_term_sx;
    casadi::SX omg_y = dz0_sx * c_yaw_sx + dz1_sx * s_yaw_sx -
                       (z0_sx * c_yaw_sx + z1_sx * s_yaw_sx) * omg_term_sx;
    casadi::SX omg_z = (z1_sx * dz0_sx - z0_sx * dz1_sx) / omg_den_sx + dyaw;

    casadi::SX DNas0_sx = ng00_sx * s_0 + ng01_sx * s_1 + ng02_sx * s_2;
    casadi::SX DNas1_sx = ng01_sx * s_0 + ng11_sx * s_1 + ng12_sx * s_2;
    casadi::SX DNas2_sx = ng02_sx * s_0 + ng12_sx * s_1 + ng22_sx * s_2;

    casadi::SX alpha = vertcat(zu0_sx, zu1_sx, zu2_sx);

    casadi::SX alpha_dot_j_sx =
        alpha(0) * j_0 + alpha(1) * j_1 + alpha(2) * j_2;
    casadi::SX alpha_dot_j_sqr_sx = alpha_dot_j_sx * alpha_dot_j_sx;
    casadi::SX j_sqr_norm_sx = j_0 * j_0 + j_1 * j_1 + j_2 * j_2;

    casadi::SX alpha_norm_3_sx = ng_den_sx;
    casadi::SX alpha_norm_5_sx = alpha_norm_3_sx * zu_sqr_norm_sx;

    casadi::SX ddz0_sx = -2 * alpha_dot_j_sx / alpha_norm_3_sx * j_0 -
                         j_sqr_norm_sx / alpha_norm_3_sx * alpha(0) +
                         3 * alpha_dot_j_sqr_sx / alpha_norm_5_sx * alpha(0) +
                         DNas0_sx;

    casadi::SX ddz1_sx = -2 * alpha_dot_j_sx / alpha_norm_3_sx * j_1 -
                         j_sqr_norm_sx / alpha_norm_3_sx * alpha(1) +
                         3 * alpha_dot_j_sqr_sx / alpha_norm_5_sx * alpha(1) +
                         DNas1_sx;

    casadi::SX ddz2_sx = -2 * alpha_dot_j_sx / alpha_norm_3_sx * j_2 -
                         j_sqr_norm_sx / alpha_norm_3_sx * alpha(2) +
                         3 * alpha_dot_j_sqr_sx / alpha_norm_5_sx * alpha(2) +
                         DNas2_sx;

    casadi::SX tmp_omg_1_sx = z0_sx * s_yaw_sx - z1_sx * c_yaw_sx;
    casadi::SX tmp_omg_2_sx = z0_sx * c_yaw_sx + z1_sx * s_yaw_sx;
    casadi::SX tmp_omg_3_sx = z1_sx * dz0_sx - z0_sx * dz1_sx;
    casadi::SX tmp_omg_4_sx = dz0_sx * s_yaw_sx - dz1_sx * c_yaw_sx;
    casadi::SX tmp_omg_5_sx = dz0_sx * c_yaw_sx + dz1_sx * s_yaw_sx;
    casadi::SX tmp_omg_6_sx = z1_sx * ddz0_sx - z0_sx * ddz1_sx;

    casadi::SX omg_dot_x =
        ddz0_sx * s_yaw_sx - ddz1_sx * c_yaw_sx -
        ddz2_sx * tmp_omg_1_sx / omg_den_sx -
        dz2_sx * tmp_omg_4_sx / omg_den_sx +
        dz2_sx * dz2_sx * tmp_omg_1_sx / (omg_den_sx * omg_den_sx);
    casadi::SX omg_dot_y =
        ddz0_sx * c_yaw_sx + ddz1_sx * s_yaw_sx -
        ddz2_sx * tmp_omg_2_sx / omg_den_sx -
        dz2_sx * tmp_omg_5_sx / omg_den_sx +
        dz2_sx * dz2_sx * tmp_omg_2_sx / (omg_den_sx * omg_den_sx);
    casadi::SX omg_dot_z = ddyaw + tmp_omg_6_sx / omg_den_sx -
                           tmp_omg_3_sx * dz2_sx / (omg_den_sx * omg_den_sx);

    casadi::SX tor_x = I_x * omg_dot_x + (I_z - I_y) * omg_y * omg_z;
    casadi::SX tor_y = I_y * omg_dot_y + (I_x - I_z) * omg_x * omg_z;
    casadi::SX tor_z = I_z * omg_dot_z + (I_y - I_x) * omg_x * omg_y;

    casadi::SX T1 = params.T_mb(0, 0) * thrust + params.T_mb(0, 1) * tor_x +
                    params.T_mb(0, 2) * tor_y + params.T_mb(0, 3) * tor_z;
    casadi::SX T2 = params.T_mb(1, 0) * thrust + params.T_mb(1, 1) * tor_x +
                    params.T_mb(1, 2) * tor_y + params.T_mb(1, 3) * tor_z;
    casadi::SX T3 = params.T_mb(2, 0) * thrust + params.T_mb(2, 1) * tor_x +
                    params.T_mb(2, 2) * tor_y + params.T_mb(2, 3) * tor_z;
    casadi::SX T4 = params.T_mb(3, 0) * thrust + params.T_mb(3, 1) * tor_x +
                    params.T_mb(3, 2) * tor_y + params.T_mb(3, 3) * tor_z;

    casadi::SX state = vertcat(std::vector<casadi::SX>{
        quat_w, quat_x, quat_y, quat_z, omg_x, omg_y, omg_z, T1, T2, T3, T4});

    fun_forward_singleThr_ =
        casadi::Function("state", {vel, acc, jer, sna, heading}, {state});

    casadi::SX jac_vel = jacobian(state, vel);
    casadi::SX jac_acc = jacobian(state, acc);
    casadi::SX jac_jer = jacobian(state, jer);
    casadi::SX jac_sna = jacobian(state, sna);
    casadi::SX jac_heading = jacobian(state, heading);

    fun_forward_singleThr_jacVel_ = casadi::Function(
        "jac_vel", {vel, acc, jer, sna, heading}, {densify(jac_vel)});
    fun_forward_singleThr_jacAcc_ = casadi::Function(
        "jac_acc", {vel, acc, jer, sna, heading}, {densify(jac_acc)});
    fun_forward_singleThr_jacJer_ = casadi::Function(
        "jac_jer", {vel, acc, jer, sna, heading}, {densify(jac_jer)});
    fun_forward_singleThr_jacSna_ = casadi::Function(
        "jac_sna", {vel, acc, jer, sna, heading}, {densify(jac_sna)});
    fun_forward_singleThr_jacHeading_ = casadi::Function(
        "jac_heading", {vel, acc, jer, sna, heading}, {densify(jac_heading)});

    return;
  }

  inline void
  forwardSingleThrAD(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                     const Eigen::Vector3d &jer, const Eigen::Vector3d &sna,
                     const Eigen::Vector3d &heading, Eigen::Vector4d &quat,
                     Eigen::Vector3d &omg, Eigen::Vector4d &thrusts,
                     Eigen::Matrix<double, 11, 3> &jacVel,
                     Eigen::Matrix<double, 11, 3> &jacAcc,
                     Eigen::Matrix<double, 11, 3> &jacJer,
                     Eigen::Matrix<double, 11, 3> &jacSna,
                     Eigen::Matrix<double, 11, 3> &jacHeading) const {
    // evaluate fun_forward_singleThr_:
    Eigen::Matrix<double, 11, 1> state;
    size_t sz_arg = fun_forward_singleThr_.sz_arg();
    size_t sz_res = fun_forward_singleThr_.sz_res();
    const double *arg[sz_arg];
    const double *input_st_0 = vel.data();
    const double *input_st_1 = acc.data();
    const double *input_st_2 = jer.data();
    const double *input_st_3 = sna.data();
    const double *input_st_4 = heading.data();

    arg[0] = (const double *)input_st_0;
    arg[1] = (const double *)input_st_1;
    arg[2] = (const double *)input_st_2;
    arg[3] = (const double *)input_st_3;
    arg[4] = (const double *)input_st_4;

    double *res_f[sz_res];
    res_f[0] = (double *)state.data();
    casadi_int *iw_f = new casadi_int[fun_forward_singleThr_.sz_iw()];
    double *w_f = new double[fun_forward_singleThr_.sz_w()];
    fun_forward_singleThr_(arg, res_f, iw_f, w_f);

    quat(0) = state(0);
    quat(1) = state(1);
    quat(2) = state(2);
    quat(3) = state(3);
    omg(0) = state(4);
    omg(1) = state(5);
    omg(2) = state(6);
    thrusts(0) = state(7);
    thrusts(1) = state(8);
    thrusts(2) = state(9);
    thrusts(3) = state(10);

    // std::cout << "ddzB_st: " << omg.transpose() << std::endl;

    delete iw_f;
    delete w_f;

    // evaluate jacVel
    jacVel.setZero();
    size_t sz_res_jacVel = fun_forward_singleThr_jacVel_.sz_res();
    double *res_jacVel[sz_res_jacVel];
    res_jacVel[0] = (double *)jacVel.data();
    casadi_int *iw_jacVel =
        new casadi_int[fun_forward_singleThr_jacVel_.sz_iw()];
    double *w_jacVel = new double[fun_forward_singleThr_jacVel_.sz_w()];
    fun_forward_singleThr_jacVel_(arg, res_jacVel, iw_jacVel, w_jacVel);

    delete iw_jacVel;
    delete w_jacVel;

    // evaluate jacAcc
    jacAcc.setZero();
    size_t sz_res_jacAcc = fun_forward_singleThr_jacAcc_.sz_res();
    double *res_jacAcc[sz_res_jacAcc];
    res_jacAcc[0] = (double *)jacAcc.data();
    casadi_int *iw_jacAcc =
        new casadi_int[fun_forward_singleThr_jacAcc_.sz_iw()];
    double *w_jacAcc = new double[fun_forward_singleThr_jacAcc_.sz_w()];
    fun_forward_singleThr_jacAcc_(arg, res_jacAcc, iw_jacAcc, w_jacAcc);

    delete iw_jacAcc;
    delete w_jacAcc;

    // evaluate jacJer
    jacJer.setZero();
    size_t sz_res_jacJer = fun_forward_singleThr_jacJer_.sz_res();
    double *res_jacJer[sz_res_jacJer];
    res_jacJer[0] = (double *)jacJer.data();
    casadi_int *iw_jacJer =
        new casadi_int[fun_forward_singleThr_jacJer_.sz_iw()];
    double *w_jacJer = new double[fun_forward_singleThr_jacJer_.sz_w()];
    fun_forward_singleThr_jacJer_(arg, res_jacJer, iw_jacJer, w_jacJer);

    delete iw_jacJer;
    delete w_jacJer;

    // evaluate jacSna
    jacSna.setZero();
    size_t sz_res_jacSna = fun_forward_singleThr_jacSna_.sz_res();
    double *res_jacSna[sz_res_jacSna];
    res_jacSna[0] = (double *)jacSna.data();
    casadi_int *iw_jacSna =
        new casadi_int[fun_forward_singleThr_jacSna_.sz_iw()];
    double *w_jacSna = new double[fun_forward_singleThr_jacSna_.sz_w()];
    fun_forward_singleThr_jacSna_(arg, res_jacSna, iw_jacSna, w_jacSna);

    delete iw_jacSna;
    delete w_jacSna;

    // evaluate jacHeading
    jacHeading.setZero();
    size_t sz_res_jacHeading = fun_forward_singleThr_jacHeading_.sz_res();
    double *res_jacHeading[sz_res_jacHeading];
    res_jacHeading[0] = (double *)jacHeading.data();
    casadi_int *iw_jacHeading =
        new casadi_int[fun_forward_singleThr_jacHeading_.sz_iw()];
    double *w_jacHeading = new double[fun_forward_singleThr_jacHeading_.sz_w()];
    fun_forward_singleThr_jacHeading_(arg, res_jacHeading, iw_jacHeading,
                                      w_jacHeading);

    delete iw_jacHeading;
    delete w_jacHeading;
  }

inline double
  computePerceptionCost(const Eigen::Vector4d &quad_orient,
                     Eigen::Matrix<double, 4, 1> &jacQuat) const {
    // evaluate fun_forward_singleThr_:
    double cost{0.0};
    size_t sz_arg = fun_perception_cost_.sz_arg();
    size_t sz_res = fun_perception_cost_.sz_res();
    const double *arg[sz_arg];
    const double *input = quad_orient.data();
    arg[0] = (const double *)input;

    double *res_cost[sz_res];
    res_cost[0] = &cost;
    casadi_int *iw_cost = new casadi_int[fun_perception_cost_.sz_iw()];
    double *w_cost = new double[fun_perception_cost_.sz_w()];
    fun_perception_cost_(arg, res_cost, iw_cost, w_cost);

    delete iw_cost;
    delete w_cost;

    // evaluate jacHeading
    jacQuat.setZero();
    size_t sz_res_jacQuat = fun_perception_cost_jacQuat_.sz_res();
    double *res_jacQuat[sz_res_jacQuat];
    res_jacQuat[0] = (double *)jacQuat.data();
    casadi_int *iw_jacQuat =
        new casadi_int[fun_perception_cost_jacQuat_.sz_iw()];
    double *w_jacQuat = new double[fun_perception_cost_jacQuat_.sz_w()];
    fun_perception_cost_jacQuat_(arg, res_jacQuat, iw_jacQuat,
                                      w_jacQuat);

    delete iw_jacQuat;
    delete w_jacQuat;

    return cost;
  }
    // double *res_cost[sz_res];
    // res_cost[0] = &cost;
    // casadi_int *iw_cost = new casadi_int[fun_perception_cost_.sz_iw()];
    // double *w_cost = new double[fun_perception_cost_.sz_w()];
    // fun_perception_cost_(arg, res_cost, iw_cost, w_cost);

    // delete iw_cost;
    // delete w_cost;


};

} // namespace drolib