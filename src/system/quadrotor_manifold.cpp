#include "drolib/system/quadrotor_manifold.hpp"

namespace drolib {
  
QuadManifold::QuadManifold() {}

QuadManifold::QuadManifold(const QuadParams params) : params_(params) {
  mat_tor_dw.setZero();
  mat_tor_dw(0, 0) = params_.inertia.x();
  mat_tor_dw(1, 1) = params_.inertia.y();
  mat_tor_dw(2, 2) = params_.inertia.z();

  inertiaGapZY = params_.inertia.z() - params_.inertia.y();
  inertiaGapXZ = params_.inertia.x() - params_.inertia.z();
  inertiaGapYX = params_.inertia.y() - params_.inertia.x();

  matTorque = params_.T_mb.rightCols(3);
  matThrust = params_.T_mb.leftCols(1);

  eps = std::numeric_limits<double>::epsilon();
}

QuadManifold::~QuadManifold() {}

bool QuadManifold::computeThrustBodyrates(const PVAJS &input, const Eigen::Vector3d& yaw, double& thrust, double& bodyrateXY, double& bodyrateZ) const {
  return true;
}

bool QuadManifold::toStateWithTrueYaw(const double t, const PVAJS &input, const Eigen::Vector3d& yaw, Setpoint &output) const {
  output.state.t = t;
  output.input.t = t;

  Eigen::Ref<Eigen::Vector3d> pos = output.state.p;
  Eigen::Ref<Eigen::Vector3d> vel = output.state.v;
  Eigen::Ref<Eigen::Vector3d> acc = output.state.a;
  Eigen::Ref<Eigen::Vector3d> jer = output.state.j;
  Eigen::Ref<Eigen::Vector3d> sna = output.state.s;
  Eigen::Ref<Eigen::Vector3d> omg = output.state.w;
  // Eigen::Ref<Eigen::Vector4d> quat = output.state.qx;
  Eigen::Ref<Eigen::Vector3d> tau = output.state.tau;

  Eigen::Vector3d& omgInput = output.input.omega; 
  Eigen::Vector4d& thrusts = output.input.thrusts; 
  double& collective_thrust = output.input.collective_thrust; 

  pos = input.col(0);
  vel = input.col(1);  
  acc = input.col(2);
  jer = input.col(3);
  sna = input.col(4);

  const double& yawAng = yaw(0);
  const double& yawRate = yaw(1);

  const Eigen::Vector3d accCmd = acc - GVEC;
  // collective_thrust = accCmd.norm() * params_.mass;
  collective_thrust = accCmd.norm(); // thrust-mass ratio
  thrusts.setConstant(NAN);

  const Eigen::Quaterniond q_c(Eigen::Quaterniond(Eigen::AngleAxis<double>(yawAng, Eigen::Vector3d::UnitZ())));
  const Eigen::Vector3d x_c = q_c * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_c = q_c * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z_B = accCmd.normalized();
  const Eigen::Vector3d x_B = (y_c.cross(z_B)).normalized();
  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  const Eigen::Quaterniond quatDes(R_W_B);
  output.state.q(quatDes);

  //TODO: should be accCmd.norm() instead of collective_thrust
  omg.x() = -1.0 / accCmd.norm() * y_B.dot(jer);
  omg.y() = 1.0 / accCmd.norm() * x_B.dot(jer);
  omg.z() = 1.0 / (y_c.cross(z_B)).norm() *
            (yawRate * x_c.dot(x_B) + omg.y() * y_c.dot(z_B));
  omgInput = omg;
  tau.setConstant(NAN);

  return true;
}
/****************************************************/
double QuadManifold::computePenalityCost(
    const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
    Eigen::Vector3d &gradTotalPos,
    Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
    Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const {
  double cost{0.0};
  Eigen::Vector3d gradPos;
  Eigen::Vector3d gradVel;
  Eigen::Vector3d gradOmg;
  Eigen::Vector4d gradQuat;
  Eigen::Vector4d gradThrusts;
  Setpoint setpoint;
  toStateWithTiltYaw(0.0, pvajs, yaw, setpoint);

  cost += addVelocityPenalities(setpoint.state.v, params, gradVel);
  cost += addRotationPenalities(setpoint.state.qx, params, gradQuat);
  cost += addBodyratePenalities(setpoint.state.w, params, gradOmg);
  cost += addThrustsPenalities(setpoint.input.thrusts, params, gradThrusts);
  cost += addBoundaryPenalities(setpoint.state.p, params, gradPos);

  backPropagate(gradPos, gradVel, gradQuat, gradOmg,
                          gradThrusts, gradTotalPos, gradTotalVel,
                          gradTotalAcc, gradTotalJer, gradTotalSna);

  return cost;
}

double QuadManifold::computeSimplePenalityCost(
    const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
    Eigen::Vector3d &gradTotalPos,
    Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
    Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const {
  double cost{0.0};
  Eigen::Vector4d gradQuat = Eigen::Vector4d::Zero();
  Eigen::Vector3d gradOmg;
  double gradThrust;
  Setpoint setpoint;
  toStateWithTiltYaw(0.0, pvajs, yaw, setpoint);

  // cost += addRotationPenalities(setpoint.state.qx, params, gradQuat);
  cost += addBodyratePenalities(setpoint.state.w, params, gradOmg);
  cost += addThrustPenality(setpoint.input.collective_thrust, params, gradThrust);

  gradTotalPos.setZero();
  gradTotalVel.setZero();
  gradTotalAcc.setZero();
  gradTotalJer.setZero();
  gradTotalSna.setZero();

  backPropagateSimple(gradQuat, gradOmg, gradThrust, gradTotalAcc, gradTotalJer);

  return cost;
}

// double QuadManifold::computeFastRacingPenalityCost(
//     const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
//     Eigen::Vector3d &gradTotalPos,
//     Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
//     Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const {

// }

double QuadManifold::computeRobustSimplePenalityCost(
    const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
    Eigen::Vector3d &gradTotalPos,
    Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
    Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const {

/*********Compute quadrotor full states*********/
  double omg_xy_sqr{0.0}, thrust{0.0};

  // Eigen::Vector3d pos = pvajs.col(0);
  // Eigen::Vector3d vel = pvajs.col(1);  
  Eigen::Vector3d acc = pvajs.col(2);
  Eigen::Vector3d jer = pvajs.col(3);
  Eigen::Vector3d sna = pvajs.col(4);
  
  a0 = acc(0);
  a1 = acc(1);
  a2 = acc(2);
  j0 = jer(0);
  j1 = jer(1);
  j2 = jer(2);
  s0 = sna(0);
  s1 = sna(1);
  s2 = sna(2);
  const double psi = yaw(0);
  const double dpsi = yaw(1);
  // const double ddpsi = yaw(2);

  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  c_psi = cos(psi);
  s_psi = sin(psi);

  // alpha = a + gzW
  alpha0 = a0;
  alpha1 = a1;
  alpha2 = a2 + G;
  alpha << alpha0, alpha1, alpha2;

  alpha_sqr0 = alpha0 * alpha0;
  alpha_sqr1 = alpha1 * alpha1;
  alpha_sqr2 = alpha2 * alpha2;
  alpha_norm_2 = alpha_sqr0 + alpha_sqr1 + alpha_sqr2;
  alpha_norm_1 = sqrt(alpha_norm_2);
  alpha_norm_3 = alpha_norm_2 * alpha_norm_1;
  alpha_norm_5 = alpha_norm_2 * alpha_norm_3;
  alpha_norm_7 = alpha_norm_2 * alpha_norm_5;
  alpha_dot_j = alpha0 * j0 + alpha1 * j1 + alpha2 * j2;
  alpha_dot_j_sqr = alpha_dot_j * alpha_dot_j;
  alpha_dot_s = alpha0 * s0 + alpha1 * s1 + alpha2 * s2;
  j_norm_2 = j0 * j0 + j1 * j1 + j2 * j2;

  mat_DNalphas_a = -sna * alpha.transpose() / alpha_norm_3 - (alpha * sna.transpose() + I33 * alpha_dot_s) / alpha_norm_3 + alpha * alpha.transpose() * 3 * alpha_dot_s / alpha_norm_5;

  // zB = N(a+gzW)
  zB0 = alpha0 / alpha_norm_1;
  zB1 = alpha1 / alpha_norm_1;
  zB2 = alpha2 / alpha_norm_1;
  zB << zB0, zB1, zB2;

  thrust = zB0 * params_.mass * a0 
                    + zB1 * params_.mass * a1
                    + zB2 * params_.mass * (a2 + G);
  zB2_1 = zB2 + 1;

  if (fabs(zB2_1) > 0.001) {
    alpha01 = alpha0 * alpha1;
    alpha12 = alpha1 * alpha2;
    alpha02 = alpha0 * alpha2;

    ng00 = (alpha_sqr1 + alpha_sqr2) / alpha_norm_3;
    ng01 = -alpha01 / alpha_norm_3;
    ng02 = -alpha02 / alpha_norm_3;
    ng11 = (alpha_sqr0 + alpha_sqr2) / alpha_norm_3;
    ng12 = -alpha12 / alpha_norm_3;
    ng22 = (alpha_sqr0 + alpha_sqr1) / alpha_norm_3;

    DN_alpha << ng00, ng01, ng02, ng01, ng11, ng12, ng02, ng12, ng22;

    // dzB
    dzB = DN_alpha * jer;
    dzB0 = dzB(0);
    dzB1 = dzB(1);
    dzB2 = dzB(2);
    dzB2_sqr = dzB2 * dzB2;

    // ddzB
    DN_alpha_s = DN_alpha * sna;
    ddzB = -jer * 2.0 * alpha_dot_j / alpha_norm_3 - alpha * j_norm_2 / alpha_norm_3 + alpha * 3.0 * alpha_dot_j_sqr / alpha_norm_5 + DN_alpha_s;
    ddzB0 = ddzB(0);
    ddzB1 = ddzB(1);
    ddzB2 = ddzB(2);

    mat_zB_a = DN_alpha;
    mat_dzB_j = DN_alpha;
    mat_ddzB_s = DN_alpha;

    mat_dzB_a = -jer * alpha.transpose() / alpha_norm_3 - (alpha * jer.transpose() + I33 * alpha_dot_j) / alpha_norm_3 + alpha * alpha.transpose() * 3 * alpha_dot_j / alpha_norm_5;
    mat_ddzB_j = -(jer * alpha.transpose() + alpha * jer.transpose() + I33 * alpha_dot_j) * 2 / alpha_norm_3 + alpha * alpha.transpose() * 6 * alpha_dot_j / alpha_norm_5;
    mat_ddzB_a = -(I33 * j_norm_2 + jer * jer.transpose() * 2.0) / alpha_norm_3 + ((jer * alpha.transpose() + alpha * jer.transpose()) * 6 * alpha_dot_j + alpha * alpha.transpose() * 3 * j_norm_2 + I33 * 3 * alpha_dot_j_sqr) / alpha_norm_5 - (alpha * alpha.transpose() * 15 * alpha_dot_j_sqr) / alpha_norm_7 + mat_DNalphas_a;

    // quaternion
    tilt_den_2 = 2.0 * (zB2_1);
    tilt_den = sqrt(tilt_den_2);
    tilt_den_3 = tilt_den_2 * tilt_den;

    tilt0 = 0.5 * tilt_den;
    tilt1 = -zB1 / tilt_den;
    tilt2 = zB0 / tilt_den;

    // bodyrate
    omg_den = zB2_1;
    omg_den_2 = omg_den * omg_den;
    omg_den_4 = omg_den_2 * omg_den_2;

    omg_term = dzB2 / omg_den;
    tmp_omg_1 = zB0 * s_psi - zB1 * c_psi;
    tmp_omg_2 = zB0 * c_psi + zB1 * s_psi;
    tmp_omg_3 = zB1 * dzB0 - zB0 * dzB1;

    omg0 = dzB0 * s_psi - dzB1 * c_psi - tmp_omg_1 * omg_term;
    omg1 = dzB0 * c_psi + dzB1 * s_psi - tmp_omg_2 * omg_term;
    omg2 = tmp_omg_3 / omg_den + dpsi;

  } else {

    double c = thrust / params_.mass;
    double c_inv = 1.0 / c;
    double c_inv_2 = c_inv * c_inv;

    omg_xy_sqr = c_inv_2 * (j0 * j0 + j1 * j1);
  }

/*********Compute costs and their gradients*********/
  gradTotalPos.setZero();
  gradTotalVel.setZero();
  gradTotalAcc.setZero();
  gradTotalJer.setZero();
  gradTotalSna.setZero();

  double cost{0.0};
  if (fabs(zB2_1) > 0.001) {
    Eigen::Vector3d gradOmg;
    double gradThrust;

    // cost += addRotationPenalities(setpoint.state.qx, params, gradQuat);
    cost += addBodyratePenalities(Eigen::Vector3d(omg0, omg1, omg2), params, gradOmg);
    cost += addThrustPenality(thrust, params, gradThrust);

    // derivative of omg w.r.t. zB
    mat_w_zB(0, 0) = -dzB2 * s_psi / omg_den;
    mat_w_zB(0, 1) = dzB2 * c_psi / omg_den;
    mat_w_zB(0, 2) = dzB2 * tmp_omg_1 / omg_den_2;
    mat_w_zB(1, 0) = -dzB2 * c_psi / omg_den;
    mat_w_zB(1, 1) = -dzB2 * s_psi / omg_den;
    mat_w_zB(1, 2) = dzB2 * tmp_omg_2 / omg_den_2;
    mat_w_zB(2, 0) = -dzB1 / omg_den;
    mat_w_zB(2, 1) = dzB0 / omg_den;
    mat_w_zB(2, 2) = -tmp_omg_3 / omg_den_2;

    // derivative of omg w.r.t. dzB
    mat_w_dzB(0, 0) = s_psi;
    mat_w_dzB(0, 1) = -c_psi;
    mat_w_dzB(0, 2) = -tmp_omg_1 / omg_den;
    mat_w_dzB(1, 0) = c_psi;
    mat_w_dzB(1, 1) = s_psi;
    mat_w_dzB(1, 2) = -tmp_omg_2 / omg_den;
    mat_w_dzB(2, 0) = zB1 / omg_den;
    mat_w_dzB(2, 1) = -zB0 / omg_den;
    mat_w_dzB(2, 2) = 0.0;

    d_Cw_a = gradOmg.transpose() * (mat_w_zB * mat_zB_a + mat_w_dzB * mat_dzB_a);
    d_Cw_j = gradOmg.transpose() * (mat_w_dzB * mat_dzB_j);

    d_Cf_a = gradThrust * params_.mass * zB.transpose();
    
    //TODO: no rotation in penality
    gradTotalAcc = d_Cw_a + d_Cf_a;
    // gradTotalAcc = d_Cq_a + d_Cw_a + d_Cf_a;
    gradTotalJer = d_Cw_j;

  } else {
    std::cout << "Singularity Handling!";
    double gradOmgXYSqr{0.0};
    // double gradOmgZSqr;
    double gradThrust{0.0};
    // gradOmgXYSqr = 0.0;
    // gradOmgZSqr = 0.0;
    // gradThrust = 0.0;
    
    double vio, vPena, vPenaD;

    //Cost 1: omg_xy_sqr
    vio = omg_xy_sqr - params.maxOmgXYSqr;
    if (smoothedL1(vio, params.smoothingEps, vPena, vPenaD)) {
      gradOmgXYSqr += params.weightOmg * vPenaD;
      cost += params.weightOmg * vPena;
    }
    //Cost 3: thrust
    // vio = (thrust - params.collectivtThrMean) * (thrust - params.collectivtThrMean) - params.collectivtThrRadiSqr;
    // if (smoothedL1(vio, params.smoothingEps, vPena, vPenaD)) {
    //   gradThrust += params.weightThr * vPenaD * 2.0 * (thrust - params.collectivtThrMean);
    //   cost += params.weightThr * vPena;
    // }
    cost += addThrustPenality(thrust, params, gradThrust);

    //Cost 1
    double c = thrust / params_.mass;
    double c_inv = 1.0 / c;
    double c_inv_2 = c_inv * c_inv;
    double c_inv_3 = c_inv_2 * c_inv;

    Eigen::Vector3d d_Cwxysqr_j = c_inv_2 * 2.0 * Eigen::Vector3d(j0, j1, 0.0);
    double d_Cwxysqr_c = - 2.0 * c_inv_3 * (j0 * j0 + j1 * j1);
    Eigen::Vector3d d_c_a = zB;
    Eigen::Vector3d d_Cwxysqr_a = d_c_a * d_Cwxysqr_c;
    //Cost 3
    d_Cf_a = gradThrust * params_.mass * zB.transpose();
    
    gradTotalAcc = d_Cwxysqr_a + d_Cf_a;
    gradTotalJer = d_Cwxysqr_j;
  }


  return cost;
}

double QuadManifold::computeRobustPenalityCost(
    const PVAJS &pvajs, const Eigen::Vector3d& yaw, const TrajParams &params,
    Eigen::Vector3d &gradTotalPos,
    Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
    Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const {

/*********Compute quadrotor full states*********/
  double omg_xy_sqr{0.0}, thrust{0.0};
  Eigen::Vector3d pos = pvajs.col(0);
  Eigen::Vector3d vel = pvajs.col(1);  
  Eigen::Vector3d acc = pvajs.col(2);
  Eigen::Vector3d jer = pvajs.col(3);
  Eigen::Vector3d sna = pvajs.col(4);
  
  a0 = acc(0);
  a1 = acc(1);
  a2 = acc(2);
  j0 = jer(0);
  j1 = jer(1);
  j2 = jer(2);
  s0 = sna(0);
  s1 = sna(1);
  s2 = sna(2);
  const double psi = yaw(0);
  const double dpsi = yaw(1);
  const double ddpsi = yaw(2);

  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  c_psi = cos(psi);
  s_psi = sin(psi);

  // alpha = a + gzW
  alpha0 = a0;
  alpha1 = a1;
  alpha2 = a2 + G;
  alpha << alpha0, alpha1, alpha2;

  alpha_sqr0 = alpha0 * alpha0;
  alpha_sqr1 = alpha1 * alpha1;
  alpha_sqr2 = alpha2 * alpha2;
  alpha_norm_2 = alpha_sqr0 + alpha_sqr1 + alpha_sqr2;
  alpha_norm_1 = sqrt(alpha_norm_2);
  alpha_norm_3 = alpha_norm_2 * alpha_norm_1;
  alpha_norm_5 = alpha_norm_2 * alpha_norm_3;
  alpha_norm_7 = alpha_norm_2 * alpha_norm_5;
  alpha_dot_j = alpha0 * j0 + alpha1 * j1 + alpha2 * j2;
  alpha_dot_j_sqr = alpha_dot_j * alpha_dot_j;
  alpha_dot_s = alpha0 * s0 + alpha1 * s1 + alpha2 * s2;
  j_norm_2 = j0 * j0 + j1 * j1 + j2 * j2;

  mat_DNalphas_a = -sna * alpha.transpose() / alpha_norm_3 - (alpha * sna.transpose() + I33 * alpha_dot_s) / alpha_norm_3 + alpha * alpha.transpose() * 3 * alpha_dot_s / alpha_norm_5;

  // zB = N(a+gzW)
  zB0 = alpha0 / alpha_norm_1;
  zB1 = alpha1 / alpha_norm_1;
  zB2 = alpha2 / alpha_norm_1;
  zB << zB0, zB1, zB2;

  thrust = zB0 * params_.mass * a0 
                    + zB1 * params_.mass * a1
                    + zB2 * params_.mass * (a2 + G);
  zB2_1 = zB2 + 1;

  if (fabs(zB2_1) > 0.001) {
    alpha01 = alpha0 * alpha1;
    alpha12 = alpha1 * alpha2;
    alpha02 = alpha0 * alpha2;

    ng00 = (alpha_sqr1 + alpha_sqr2) / alpha_norm_3;
    ng01 = -alpha01 / alpha_norm_3;
    ng02 = -alpha02 / alpha_norm_3;
    ng11 = (alpha_sqr0 + alpha_sqr2) / alpha_norm_3;
    ng12 = -alpha12 / alpha_norm_3;
    ng22 = (alpha_sqr0 + alpha_sqr1) / alpha_norm_3;

    DN_alpha << ng00, ng01, ng02, ng01, ng11, ng12, ng02, ng12, ng22;

    // dzB
    dzB = DN_alpha * jer;
    dzB0 = dzB(0);
    dzB1 = dzB(1);
    dzB2 = dzB(2);
    dzB2_sqr = dzB2 * dzB2;

    // ddzB
    DN_alpha_s = DN_alpha * sna;
    ddzB = -jer * 2.0 * alpha_dot_j / alpha_norm_3 - alpha * j_norm_2 / alpha_norm_3 + alpha * 3.0 * alpha_dot_j_sqr / alpha_norm_5 + DN_alpha_s;
    ddzB0 = ddzB(0);
    ddzB1 = ddzB(1);
    ddzB2 = ddzB(2);

    mat_zB_a = DN_alpha;
    mat_dzB_j = DN_alpha;
    mat_ddzB_s = DN_alpha;

    mat_dzB_a = -jer * alpha.transpose() / alpha_norm_3 - (alpha * jer.transpose() + I33 * alpha_dot_j) / alpha_norm_3 + alpha * alpha.transpose() * 3 * alpha_dot_j / alpha_norm_5;
    mat_ddzB_j = -(jer * alpha.transpose() + alpha * jer.transpose() + I33 * alpha_dot_j) * 2 / alpha_norm_3 + alpha * alpha.transpose() * 6 * alpha_dot_j / alpha_norm_5;
    mat_ddzB_a = -(I33 * j_norm_2 + jer * jer.transpose() * 2.0) / alpha_norm_3 + ((jer * alpha.transpose() + alpha * jer.transpose()) * 6 * alpha_dot_j + alpha * alpha.transpose() * 3 * j_norm_2 + I33 * 3 * alpha_dot_j_sqr) / alpha_norm_5 - (alpha * alpha.transpose() * 15 * alpha_dot_j_sqr) / alpha_norm_7 + mat_DNalphas_a;

    // quaternion
    tilt_den_2 = 2.0 * (zB2_1);
    tilt_den = sqrt(tilt_den_2);
    tilt_den_3 = tilt_den_2 * tilt_den;

    tilt0 = 0.5 * tilt_den;
    tilt1 = -zB1 / tilt_den;
    tilt2 = zB0 / tilt_den;

    quat_tmp(0) = tilt0 * c_half_psi;
    quat_tmp(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
    quat_tmp(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
    quat_tmp(3) = tilt0 * s_half_psi;

    // bodyrate
    omg_den = zB2_1;
    omg_den_2 = omg_den * omg_den;
    omg_den_4 = omg_den_2 * omg_den_2;

    omg_term = dzB2 / omg_den;
    tmp_omg_1 = zB0 * s_psi - zB1 * c_psi;
    tmp_omg_2 = zB0 * c_psi + zB1 * s_psi;
    tmp_omg_3 = zB1 * dzB0 - zB0 * dzB1;

    omg_tmp(0) = dzB0 * s_psi - dzB1 * c_psi - tmp_omg_1 * omg_term;
    omg_tmp(1) = dzB0 * c_psi + dzB1 * s_psi - tmp_omg_2 * omg_term;
    omg_tmp(2) = tmp_omg_3 / omg_den + dpsi;

    // derivative of torque w.r.t. omg
    mat_tor_w << 0.0, inertiaGapZY * omg_tmp(2), inertiaGapZY * omg_tmp(1),
                  inertiaGapXZ * omg_tmp(2), 0.0, inertiaGapXZ * omg_tmp(0),
                  inertiaGapYX * omg_tmp(1), inertiaGapYX * omg_tmp(0), 0.0;

    // body acceleration
    tmp_omg_4 = dzB0 * s_psi - dzB1 * c_psi;
    tmp_omg_5 = dzB0 * c_psi + dzB1 * s_psi;
    tmp_omg_6 = zB1 * ddzB0 - zB0 * ddzB1;

    omg_dot(0) = ddzB0 * s_psi - ddzB1 * c_psi - ddzB2 * tmp_omg_1 / omg_den - dzB2 * tmp_omg_4 / omg_den + dzB2 * dzB2 * tmp_omg_1 / omg_den_2;
    omg_dot(1) = ddzB0 * c_psi + ddzB1 * s_psi - ddzB2 * tmp_omg_2 / omg_den - dzB2 * tmp_omg_5 / omg_den + dzB2 * dzB2 * tmp_omg_2 / omg_den_2;
    omg_dot(2) = tmp_omg_6 / omg_den - tmp_omg_3 * dzB2 / omg_den_2 + ddpsi;

    //TODO: the correct expression leads to a worse result...
    tau_tmp(0) = params_.inertia.x() * omg_dot(0) + inertiaGapZY * omg_tmp(1) * omg_tmp(2);
    tau_tmp(1) = params_.inertia.y() * omg_dot(1) + inertiaGapXZ * omg_tmp(0) * omg_tmp(2);
    tau_tmp(2) = params_.inertia.z() * omg_dot(2) + inertiaGapYX * omg_tmp(0) * omg_tmp(1);
    // tau_tmp.setZero(); //TODO: in this case, the collective thrust is constrained indeed

    thrusts_tmp = params_.T_mb * (Eigen::Vector4d() << thrust, tau_tmp).finished();

  } else {

    double c = thrust / params_.mass;
    double c_inv = 1.0 / c;
    double c_inv_2 = c_inv * c_inv;

    omg_xy_sqr = c_inv_2 * (j0 * j0 + j1 * j1);
  }

  
/*********Compute costs and their gradients*********/
  gradTotalPos.setZero();
  gradTotalVel.setZero();
  gradTotalAcc.setZero();
  gradTotalJer.setZero();
  gradTotalSna.setZero();

  double cost{0.0};
  if (fabs(zB2_1) > 0.001) {
    Eigen::Vector3d gradPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d gradVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gradOmg = Eigen::Vector3d::Zero();
    Eigen::Vector4d gradQuat = Eigen::Vector4d::Zero();
    Eigen::Vector4d gradThrusts = Eigen::Vector4d::Zero();

    cost += addVelocityPenalities(vel, params, gradVel);
    cost += addRotationPenalities(quat_tmp, params, gradQuat);
    cost += addBodyratePenalities(omg_tmp, params, gradOmg);
    cost += addThrustsPenalities(thrusts_tmp, params, gradThrusts);
    cost += addBoundaryPenalities(pos, params, gradPos);

    backPropagate(gradPos, gradVel, gradQuat, gradOmg,
                            gradThrusts, gradTotalPos, gradTotalVel,
                            gradTotalAcc, gradTotalJer, gradTotalSna);

  } else {
    // std::cout << "Singularity Handling!";
    double gradOmgXYSqr{0.0};
    // double gradOmgZSqr;
    double gradThrust{0.0};
    // gradOmgXYSqr = 0.0;
    // gradOmgZSqr = 0.0;
    // gradThrust = 0.0;
    
    double vio, vPena, vPenaD;
    //Cost 1: omg_xy_sqr
    vio = omg_xy_sqr - params.maxOmgXYSqr;
    if (smoothedL1(vio, params.smoothingEps, vPena, vPenaD)) {
      gradOmgXYSqr += params.weightOmg * vPenaD;
      cost += params.weightOmg * vPena;
    }

    cost += addThrustPenality(thrust, params, gradThrust);

    //Cost 1
    double c = thrust / params_.mass;
    double c_inv = 1.0 / c;
    double c_inv_2 = c_inv * c_inv;
    double c_inv_3 = c_inv_2 * c_inv;

    Eigen::Vector3d d_Cwxysqr_j = c_inv_2 * 2.0 * Eigen::Vector3d(j0, j1, 0.0);
    double d_Cwxysqr_c = - 2.0 * c_inv_3 * (j0 * j0 + j1 * j1);
    Eigen::Vector3d d_c_a = zB;
    Eigen::Vector3d d_Cwxysqr_a = d_c_a * d_Cwxysqr_c;
    //Cost 3
    d_Cf_a = gradThrust * params_.mass * zB.transpose();
    
    gradTotalAcc = d_Cwxysqr_a + d_Cf_a;
    gradTotalJer = d_Cwxysqr_j;
  }


  return cost;
}

double QuadManifold::addThrustPenality(const double thrust, 
                                  const TrajParams &params,
                                  double& gradThrust) const {
  double penalty{0.0};     
  gradThrust = 0.0;
  if (params.weightThr <= 1.0e-6) {
    return penalty;
  }  
  // thrust variable here is already mass-normalized and therefore it needs to be scaled by a mass factor
  double v, vPena, vPenaD;
  v = (thrust - params.collectivtThrMean) * (thrust - params.collectivtThrMean) - params.collectivtThrRadiSqr;
  if (smoothedL1(v, params.smoothingEps, vPena, vPenaD)) {
    gradThrust += params.weightThr * vPenaD * 2.0 * (thrust - params.collectivtThrMean);
    penalty += params.weightThr * vPena;
  }

  return penalty;                       
}

double QuadManifold::addThrustsPenalities(const Eigen::Vector4d &thrusts, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector4d> gradThrusts) const {
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

double QuadManifold::addVelocityPenalities(const Eigen::Vector3d &vel, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector3d> gradVel) const {
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

double QuadManifold::addBodyratePenalities(const Eigen::Vector3d &omg, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector3d> gradOmg) const {
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

double QuadManifold::addRotationPenalities(const Eigen::Vector4d &quat, 
                                  const TrajParams &params,
                                  Eigen::Ref<Eigen::Vector4d> gradQuat) const {
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



double QuadManifold::addBoundaryPenalities(const Eigen::Vector3d &pos, 
                                      const TrajParams &params,
                                      Eigen::Ref<Eigen::Vector3d> gradPos) const {
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

bool QuadManifold::smoothedL1(const double &x, const double &mu, double &f,
                              double &df) const {
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
/****************************************************/
bool QuadManifold::toStateWithTiltYaw(const double t, const PVAJS &input, const Eigen::Vector3d& yaw, Setpoint &output) const {
  output.state.t = t;
  output.input.t = t;

  Eigen::Ref<Eigen::Vector3d> pos = output.state.p;
  Eigen::Ref<Eigen::Vector3d> vel = output.state.v;
  Eigen::Ref<Eigen::Vector3d> acc = output.state.a;
  Eigen::Ref<Eigen::Vector3d> jer = output.state.j;
  Eigen::Ref<Eigen::Vector3d> sna = output.state.s;
  Eigen::Ref<Eigen::Vector3d> omg = output.state.w;
  Eigen::Ref<Eigen::Vector4d> quat = output.state.qx;
  Eigen::Ref<Eigen::Vector3d> tau = output.state.tau;

  Eigen::Vector3d& omgInput = output.input.omega; 
  Eigen::Vector4d& thrusts = output.input.thrusts; 
  double& thrust = output.input.collective_thrust; 

  pos = input.col(0);
  vel = input.col(1);  
  acc = input.col(2);
  jer = input.col(3);
  sna = input.col(4);

  a0 = acc(0);
  a1 = acc(1);
  a2 = acc(2);
  j0 = jer(0);
  j1 = jer(1);
  j2 = jer(2);
  s0 = sna(0);
  s1 = sna(1);
  s2 = sna(2);
  const double psi = yaw(0);
  const double dpsi = yaw(1);
  const double ddpsi = yaw(2);

  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  c_psi = cos(psi);
  s_psi = sin(psi);

  // alpha = a + gzW
  alpha0 = a0;
  alpha1 = a1;
  alpha2 = a2 + G;
  alpha << alpha0, alpha1, alpha2;

  alpha_sqr0 = alpha0 * alpha0;
  alpha_sqr1 = alpha1 * alpha1;
  alpha_sqr2 = alpha2 * alpha2;
  alpha_norm_2 = alpha_sqr0 + alpha_sqr1 + alpha_sqr2;
  // if (alpha_norm_2 < 1.0e-6) {
  //   return false;
  // }
  alpha_norm_1 = sqrt(alpha_norm_2);



  alpha_norm_3 = alpha_norm_2 * alpha_norm_1;
  alpha_norm_5 = alpha_norm_2 * alpha_norm_3;
  alpha_norm_7 = alpha_norm_2 * alpha_norm_5;
  alpha_dot_j = alpha0 * j0 + alpha1 * j1 + alpha2 * j2;
  alpha_dot_j_sqr = alpha_dot_j * alpha_dot_j;
  alpha_dot_s = alpha0 * s0 + alpha1 * s1 + alpha2 * s2;
  j_norm_2 = j0 * j0 + j1 * j1 + j2 * j2;

  mat_DNalphas_a = -sna * alpha.transpose() / alpha_norm_3 - (alpha * sna.transpose() + I33 * alpha_dot_s) / alpha_norm_3 + alpha * alpha.transpose() * 3 * alpha_dot_s / alpha_norm_5;

  // zB = N(a+gzW)
  zB0 = alpha0 / alpha_norm_1;
  zB1 = alpha1 / alpha_norm_1;
  zB2 = alpha2 / alpha_norm_1;
  zB << zB0, zB1, zB2;
  zB2_1 = zB2 + 1;

  // collective thrust: thrust-mass ratio
  // thrust = alpha_norm_1;

  thrust = zB0 * params_.mass * a0 
              + zB1 * params_.mass * a1
              + zB2 * params_.mass * (a2 + G);

  // if (thrust < 1.0e-3) {
  //   //Free fall situation

  //   //TODO: compute the quaterinon and bodyrate commands
  //   // quat
  //   // omg
  //   // omgInput
  //   // tau
  //   // thrusts
  //   const Eigen::Quaterniond q_heading(quaternionAtUnitZ(yaw(0)));
  //   const Eigen::Quaterniond q_att = q_tilt_last_ * q_heading;
  //   quat(0) = q_att.w();
  //   quat(1) = q_att.x();
  //   quat(2) = q_att.y();
  //   quat(3) = q_att.z();

  //   const Eigen::Vector3d body_jerk = q_att.inverse() * jer;
  //   omg = Eigen::Vector3d(-1.0 / thrust * body_jerk[1], 1.0 / thrust * body_jerk[0], yaw(1)); // (0, 0, yawrate)
  //   // thrusts.setConstant(0.0);

  //   return false;
  // }

  // if (fabs(zB2_1) < 1.0e-4) {
  //   //TODO: Flip flight situation
  //   return false;
  // }

  alpha01 = alpha0 * alpha1;
  alpha12 = alpha1 * alpha2;
  alpha02 = alpha0 * alpha2;

  ng00 = (alpha_sqr1 + alpha_sqr2) / alpha_norm_3;
  ng01 = -alpha01 / alpha_norm_3;
  ng02 = -alpha02 / alpha_norm_3;
  ng11 = (alpha_sqr0 + alpha_sqr2) / alpha_norm_3;
  ng12 = -alpha12 / alpha_norm_3;
  ng22 = (alpha_sqr0 + alpha_sqr1) / alpha_norm_3;

  DN_alpha << ng00, ng01, ng02, ng01, ng11, ng12, ng02, ng12, ng22;

  // dzB
  dzB = DN_alpha * jer;
  dzB0 = dzB(0);
  dzB1 = dzB(1);
  dzB2 = dzB(2);
  dzB2_sqr = dzB2 * dzB2;

  // ddzB
  DN_alpha_s = DN_alpha * sna;
  ddzB = -jer * 2.0 * alpha_dot_j / alpha_norm_3 - alpha * j_norm_2 / alpha_norm_3 + alpha * 3.0 * alpha_dot_j_sqr / alpha_norm_5 + DN_alpha_s;
  ddzB0 = ddzB(0);
  ddzB1 = ddzB(1);
  ddzB2 = ddzB(2);

  mat_zB_a = DN_alpha;
  mat_dzB_j = DN_alpha;
  mat_ddzB_s = DN_alpha;

  mat_dzB_a = -jer * alpha.transpose() / alpha_norm_3 - (alpha * jer.transpose() + I33 * alpha_dot_j) / alpha_norm_3 + alpha * alpha.transpose() * 3 * alpha_dot_j / alpha_norm_5;
  mat_ddzB_j = -(jer * alpha.transpose() + alpha * jer.transpose() + I33 * alpha_dot_j) * 2 / alpha_norm_3 + alpha * alpha.transpose() * 6 * alpha_dot_j / alpha_norm_5;
  mat_ddzB_a = -(I33 * j_norm_2 + jer * jer.transpose() * 2.0) / alpha_norm_3 + ((jer * alpha.transpose() + alpha * jer.transpose()) * 6 * alpha_dot_j + alpha * alpha.transpose() * 3 * j_norm_2 + I33 * 3 * alpha_dot_j_sqr) / alpha_norm_5 - (alpha * alpha.transpose() * 15 * alpha_dot_j_sqr) / alpha_norm_7 + mat_DNalphas_a;

  // quaternion
  tilt_den_2 = 2.0 * (zB2_1);
  tilt_den = sqrt(tilt_den_2);
  tilt_den_3 = tilt_den_2 * tilt_den;

  tilt0 = 0.5 * tilt_den;
  tilt1 = -zB1 / tilt_den;
  tilt2 = zB0 / tilt_den;
  // q_tilt_last_ = Eigen::Quaterniond(tilt0, tilt1, tilt2, 0.0);
  
  quat(0) = tilt0 * c_half_psi;
  quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
  quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
  quat(3) = tilt0 * s_half_psi;

  // bodyrate
  omg_den = zB2_1;
  omg_den_2 = omg_den * omg_den;
  omg_den_4 = omg_den_2 * omg_den_2;

  omg_term = dzB2 / omg_den;
  tmp_omg_1 = zB0 * s_psi - zB1 * c_psi;
  tmp_omg_2 = zB0 * c_psi + zB1 * s_psi;
  tmp_omg_3 = zB1 * dzB0 - zB0 * dzB1;
  omg(0) = dzB0 * s_psi - dzB1 * c_psi - tmp_omg_1 * omg_term;
  omg(1) = dzB0 * c_psi + dzB1 * s_psi - tmp_omg_2 * omg_term;
  omg(2) = tmp_omg_3 / omg_den + dpsi;
  omgInput = omg;

  // derivative of torque w.r.t. omg
  mat_tor_w << 0.0, inertiaGapZY * omg(2), inertiaGapZY * omg(1),
                inertiaGapXZ * omg(2), 0.0, inertiaGapXZ * omg(0),
                inertiaGapYX * omg(1), inertiaGapYX * omg(0), 0.0;

  // body acceleration
  tmp_omg_4 = dzB0 * s_psi - dzB1 * c_psi;
  tmp_omg_5 = dzB0 * c_psi + dzB1 * s_psi;
  tmp_omg_6 = zB1 * ddzB0 - zB0 * ddzB1;

  omg_dot(0) = ddzB0 * s_psi - ddzB1 * c_psi - ddzB2 * tmp_omg_1 / omg_den - dzB2 * tmp_omg_4 / omg_den + dzB2 * dzB2 * tmp_omg_1 / omg_den_2;
  omg_dot(1) = ddzB0 * c_psi + ddzB1 * s_psi - ddzB2 * tmp_omg_2 / omg_den - dzB2 * tmp_omg_5 / omg_den + dzB2 * dzB2 * tmp_omg_2 / omg_den_2;
  omg_dot(2) = tmp_omg_6 / omg_den - tmp_omg_3 * dzB2 / omg_den_2 + ddpsi;

  tau(0) = params_.inertia.x() * omg_dot(0) + inertiaGapZY * omg(1) * omg(2);
  tau(1) = params_.inertia.y() * omg_dot(1) + inertiaGapXZ * omg(0) * omg(2);
  tau(2) = params_.inertia.z() * omg_dot(2) + inertiaGapYX * omg(0) * omg(1);

  // thrusts = params_.T_mb * (Eigen::Vector4d() << thrust * params_.mass, tau).finished();
  thrusts = params_.T_mb * (Eigen::Vector4d() << thrust, tau).finished();
  return true;
}

void QuadManifold::backPropagate(
    const Eigen::Vector3d &gradPos, const Eigen::Vector3d &gradVel,
    const Eigen::Vector4d &gradQuat, const Eigen::Vector3d &gradOmg,
    const Eigen::Vector4d &gradThr, Eigen::Vector3d &gradTotalPos,
    Eigen::Vector3d &gradTotalVel, Eigen::Vector3d &gradTotalAcc,
    Eigen::Vector3d &gradTotalJer, Eigen::Vector3d &gradTotalSna) const {
  calcJacobian();

  // contribution of rotation penalty
  tmp_quat_1 = 1.0 / tilt_den;
  tmp_quat_2 = 1.0 / tilt_den_3;
  d_Cq_zB(0) = gradQuat(1) * (tmp_quat_1 * s_half_psi) +
              gradQuat(2) * (tmp_quat_1 * c_half_psi);
  d_Cq_zB(1) = gradQuat(1) * (-tmp_quat_1 * c_half_psi) +
              gradQuat(2) * (tmp_quat_1 * s_half_psi);
  d_Cq_zB(2) =
      gradQuat(0) * (0.5 * tmp_quat_1 * c_half_psi) +
      gradQuat(1) *
          (tmp_quat_2 * zB1 * c_half_psi - tmp_quat_2 * zB0 * s_half_psi) +
      gradQuat(2) *
          (-tmp_quat_2 * zB0 * c_half_psi - tmp_quat_2 * zB1 * s_half_psi) +
      gradQuat(3) * (0.5 * tmp_quat_1 * s_half_psi);
  d_Cq_a = d_Cq_zB.transpose() * mat_zB_a;

  // contribution of bodyrate penalty
  d_Cw_a = gradOmg.transpose() * (mat_w_zB * mat_zB_a + mat_w_dzB * mat_dzB_a);
  d_Cw_j = gradOmg.transpose() * (mat_w_dzB * mat_dzB_j);

  // contribution of thrust penalty
  gradTorque = gradThr.transpose() * matTorque;
  gradCollectiveThr = gradThr.dot(matThrust);

  d_Cf_a.setZero();
  d_Cf_a += gradTorque.transpose() *
          (mat_tor_dw * (mat_dw_zB * mat_zB_a + mat_dw_dzB * mat_dzB_a +
                          mat_dw_ddzB * mat_ddzB_a) +
              mat_tor_w * (mat_w_zB * mat_zB_a + mat_w_dzB * mat_dzB_a));
  d_Cf_a += gradCollectiveThr * params_.mass * zB.transpose();

  d_Cf_j = gradTorque.transpose() *
          (mat_tor_dw * (mat_dw_dzB * mat_dzB_j + mat_dw_ddzB * mat_ddzB_j) +
          mat_tor_w * (mat_w_dzB * mat_dzB_j));

  d_Cf_s = gradTorque.transpose() * mat_tor_dw * mat_dw_ddzB * mat_ddzB_s;

  // sum up all penalty contributions

  gradTotalPos = gradPos;
  gradTotalVel = gradVel;
  gradTotalAcc = d_Cq_a + d_Cw_a + d_Cf_a;
  gradTotalJer = d_Cw_j + d_Cf_j;
  gradTotalSna = d_Cf_s;

  return;
}

void QuadManifold::backPropagateSimple(
  const Eigen::Vector4d &gradQuat,
  const Eigen::Vector3d &gradOmg,
  const double gradThr, 
  Eigen::Vector3d &gradTotalAcc,
  Eigen::Vector3d &gradTotalJer) const {

  calcJacobian();

  // contribution of bodyrate penalty
  d_Cw_a = gradOmg.transpose() * (mat_w_zB * mat_zB_a + mat_w_dzB * mat_dzB_a);
  d_Cw_j = gradOmg.transpose() * (mat_w_dzB * mat_dzB_j);

  d_Cf_a = gradThr * params_.mass * zB.transpose();
  
  //TODO: no rotation in penality
  gradTotalAcc = d_Cw_a + d_Cf_a;
  gradTotalJer = d_Cw_j;
}

void QuadManifold::calcJacobian(void) const {
  // derivative of omg w.r.t. zB
  mat_w_zB(0, 0) = -dzB2 * s_psi / omg_den;
  mat_w_zB(0, 1) = dzB2 * c_psi / omg_den;
  mat_w_zB(0, 2) = dzB2 * tmp_omg_1 / omg_den_2;
  mat_w_zB(1, 0) = -dzB2 * c_psi / omg_den;
  mat_w_zB(1, 1) = -dzB2 * s_psi / omg_den;
  mat_w_zB(1, 2) = dzB2 * tmp_omg_2 / omg_den_2;
  mat_w_zB(2, 0) = -dzB1 / omg_den;
  mat_w_zB(2, 1) = dzB0 / omg_den;
  mat_w_zB(2, 2) = -tmp_omg_3 / omg_den_2;

  // derivative of omg w.r.t. dzB
  mat_w_dzB(0, 0) = s_psi;
  mat_w_dzB(0, 1) = -c_psi;
  mat_w_dzB(0, 2) = -tmp_omg_1 / omg_den;

  mat_w_dzB(1, 0) = c_psi;
  mat_w_dzB(1, 1) = s_psi;
  mat_w_dzB(1, 2) = -tmp_omg_2 / omg_den;

  mat_w_dzB(2, 0) = zB1 / omg_den;
  mat_w_dzB(2, 1) = -zB0 / omg_den;
  mat_w_dzB(2, 2) = 0.0;

  // derivative of omg_dot w.r.t. zB
  // d_dwx_zB
  const double d_dw0_zB0 = -ddzB2 * s_psi / omg_den + dzB2_sqr * s_psi / omg_den_2;
  const double d_dw0_zB1 = ddzB2 * c_psi / omg_den - dzB2_sqr * c_psi / omg_den_2;
  const double d_dw0_zB2 = ddzB2 * tmp_omg_1 / omg_den_2 +
                      dzB2 * tmp_omg_4 / omg_den_2 -
                      dzB2_sqr * 2 * omg_den * tmp_omg_1 / omg_den_4;
  // d_dwy_zB
  const double d_dw1_zB0 = -ddzB2 * c_psi / omg_den + dzB2_sqr * c_psi / omg_den_2;
  const double d_dw1_zB1 = -ddzB2 * s_psi / omg_den + dzB2_sqr * s_psi / omg_den_2;
  const double d_dw1_zB2 = ddzB2 * tmp_omg_2 / omg_den_2 +
                      dzB2 * tmp_omg_5 / omg_den_2 -
                      dzB2_sqr * 2 * omg_den * tmp_omg_2 / omg_den_4;

  // d_dwy_zB
  const double d_dw2_zB0 = -ddzB1 / omg_den + dzB1 * dzB2 / omg_den_2;
  const double d_dw2_zB1 = ddzB0 / omg_den - dzB0 * dzB2 / omg_den_2;
  const double d_dw2_zB2 =
      -tmp_omg_6 / omg_den_2 + 2 * omg_den * tmp_omg_3 * dzB2 / omg_den_4;

  mat_dw_zB << d_dw0_zB0, d_dw0_zB1, d_dw0_zB2, d_dw1_zB0, d_dw1_zB1, d_dw1_zB2,
      d_dw2_zB0, d_dw2_zB1, d_dw2_zB2;

  // derivative of omg_dot w.r.t. dzB
  // d_dwx_dzB
  const double d_dw0_dzB0 = -dzB2 * s_psi / omg_den;
  const double d_dw0_dzB1 = dzB2 * c_psi / omg_den;
  const double d_dw0_dzB2 = -tmp_omg_4 / omg_den + 2 * dzB2 * tmp_omg_1 / omg_den_2;

  // d_dwy_dzB
  const double d_dw1_dzB0 = -dzB2 * c_psi / omg_den;
  const double d_dw1_dzB1 = -dzB2 * s_psi / omg_den;
  const double d_dw1_dzB2 = -tmp_omg_5 / omg_den + 2 * dzB2 * tmp_omg_2 / omg_den_2;

  // d_dwz_dzB
  const double d_dw2_dzB0 = -zB1 * dzB2 / omg_den_2;
  const double d_dw2_dzB1 = zB0 * dzB2 / omg_den_2;
  const double d_dw2_dzB2 = -tmp_omg_3 / omg_den_2;

  mat_dw_dzB << d_dw0_dzB0, d_dw0_dzB1, d_dw0_dzB2, d_dw1_dzB0, d_dw1_dzB1,
      d_dw1_dzB2, d_dw2_dzB0, d_dw2_dzB1, d_dw2_dzB2;

  // derivative of omg_dot w.r.t. ddzB
  // d_dwx_ddzB
  const double d_dw0_ddzB0 = s_psi;
  const double d_dw0_ddzB1 = -c_psi;
  const double d_dw0_ddzB2 = -tmp_omg_1 / omg_den;

  // d_dwy_ddzB
  const double d_dw1_ddzB0 = c_psi;
  const double d_dw1_ddzB1 = s_psi;
  const double d_dw1_ddzB2 = -tmp_omg_2 / omg_den;

  // d_dwz_ddzB
  const double d_dw2_ddzB0 = zB1 / omg_den;
  const double d_dw2_ddzB1 = -zB0 / omg_den;
  const double d_dw2_ddzB2 = 0.0;

  mat_dw_ddzB << d_dw0_ddzB0, d_dw0_ddzB1, d_dw0_ddzB2, d_dw1_ddzB0,
      d_dw1_ddzB1, d_dw1_ddzB2, d_dw2_ddzB0, d_dw2_ddzB1, d_dw2_ddzB2;

  return;
}

}  // namespace drolib