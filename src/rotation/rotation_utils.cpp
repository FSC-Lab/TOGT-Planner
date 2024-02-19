#include "drolib/rotation/rotation_utils.h"

namespace drolib {

Eigen::Vector3d rad2deg(const Eigen::Vector3d& rad) {
  return Eigen::Vector3d(rad2deg(rad.x()), rad2deg(rad.y()), rad2deg(rad.z()));
}

Eigen::Vector3d deg2rad(const Eigen::Vector3d& deg) {
  return Eigen::Vector3d(deg2rad(deg.x()), deg2rad(deg.y()), deg2rad(deg.z()));
}

Eigen::Quaterniond quatDelta(const Eigen::Vector3d& theta) {
  const Eigen::Vector3d half_theta(theta * 0.5);
  Eigen::Quaterniond dq;
  dq.w() = 1.0f;
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d v_sqew;
  v_sqew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return v_sqew;
}

Eigen::Vector3d quaternionToEulerAnglesRPY(const Eigen::Quaterniond& q) {
  // Eigen::Vector3d euler_angles;
  // euler_angles(0) =
  //     atan2(2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
  //           q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  // euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
  // euler_angles(2) =
  //     atan2(2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
  //           q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  // return euler_angles;

  Eigen::Vector3d rpy;
  // order
  // https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
  Eigen::Matrix<double, 4, 1> coeff = q.coeffs();
  const double x = coeff(0);
  const double y = coeff(1);
  const double z = coeff(2);
  const double w = coeff(3);

  const double y_sqr = y * y;

  const double t0 = +2.0 * (w * x + y * z);
  const double t1 = +1.0 - 2.0 * (x * x + y_sqr);

  rpy[0] = atan2(t0, t1);

  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  rpy[1] = asin(t2);

  const double t3 = +2.0 * (w * z + x * y);
  const double t4 = +1.0 - 2.0 * (y_sqr + z * z);
  rpy[2] = atan2(t3, t4);
  return rpy;
}

Eigen::Vector3d quaternionToAngleAxis(const Eigen::Quaterniond& q) {
  Eigen::Vector3d rvec;
  Eigen::AngleAxis<double> angleaxis(q);
  rvec = angleaxis.angle() * angleaxis.axis();
  return rvec;
}

Eigen::Quaterniond eulerAnglesRPYToQuaternion(const Eigen::Vector3d& rpy) {
  Eigen::Quaterniond q;
  const double r = rpy(0) / 2.0;
  const double p = rpy(1) / 2.0;
  const double y = rpy(2) / 2.0;
  q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
  return q;
}

Eigen::Quaterniond angleAxisToQuaternion(const Eigen::Vector3d& rvec) {
  const double angle = rvec.norm();
  const Eigen::Vector3d axis = rvec / angle;
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

Eigen::Vector3d rotationMatrixToEulerAnglesRPY(const Eigen::Matrix3d& R) {
  Eigen::Vector3d rpy;
  rpy(0) = atan2(R(2, 1), R(2, 2));
  rpy(1) = -atan2(R(2, 0), sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0)));
  rpy(2) = atan2(R(1, 0), R(0, 0));
  return rpy;
}

Eigen::Matrix3d eulerAnglesRPYToRotationMatrix(const Eigen::Vector3d& rpy) {
  const double r = rpy(0);
  const double p = rpy(1);
  const double y = rpy(2);

  Eigen::Matrix3d R;
  R(0, 0) = cos(y) * cos(p);
  R(1, 0) = sin(y) * cos(p);
  R(2, 0) = -sin(p);

  R(0, 1) = cos(y) * sin(p) * sin(r) - sin(y) * cos(r);
  R(1, 1) = sin(y) * sin(p) * sin(r) + cos(y) * cos(r);
  R(2, 1) = cos(p) * sin(r);

  R(0, 2) = cos(y) * sin(p) * cos(r) + sin(y) * sin(r);
  R(1, 2) = sin(y) * sin(p) * cos(r) - cos(y) * sin(r);
  R(2, 2) = cos(p) * cos(r);

  return R;
}

Eigen::Quaterniond quaternionAtUnitX(const double roll) {
  Eigen::Quaterniond qx;
  qx.setIdentity();
  const double c_half = cos(0.5 * roll);
  const double s_half = sin(0.5 * roll);
  qx.w() = c_half;
  qx.x() = s_half;
  return qx;
}

Eigen::Quaterniond quaternionAtUnitY(const double pitch) {
  Eigen::Quaterniond qy;
  qy.setIdentity();
  const double c_half = cos(0.5 * pitch);
  const double s_half = sin(0.5 * pitch);
  qy.w() = c_half;
  qy.y() = s_half;
  return qy;
}

Eigen::Quaterniond quaternionAtUnitZ(const double yaw) {
  Eigen::Quaterniond qz;
  qz.setIdentity();
  const double c_half = cos(0.5 * yaw);
  const double s_half = sin(0.5 * yaw);
  qz.w() = c_half;
  qz.z() = s_half;
  return qz;
}

Eigen::Quaterniond quaternionFromUnitZToV(const Eigen::Vector3d &v) {
  // FromTwoVectors()
  Eigen::Vector3d v_norm = v / v.norm();
  Eigen::Quaterniond q;
  const double tilt_den = sqrt(2.0 * (1.0 + v_norm.z()));
  q.w() = 0.5 * tilt_den;
  q.x() = -v_norm.y() / tilt_den;
  q.y() = v_norm.x() / tilt_den;
  q.z() = 0.0;
  return q;
}

double quaternionToYaw(const Eigen::Quaterniond &q, const double yaw) {
  const Eigen::Vector3d x_b = q * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d x_proj = Eigen::Vector3d(x_b.x(), x_b.y(), 0);
  if (x_proj.norm() < 1e-3) return yaw;
  const Eigen::Vector3d x_proj_norm = x_proj.normalized();
  const Eigen::Vector3d cross = Eigen::Vector3d::UnitX().cross(x_proj_norm);
  const double angle = asin(cross.z());
  if (x_proj_norm.x() >= 0.0) return angle;
  if (x_proj_norm.y() >= 0.0) return M_PI - angle;
  return -M_PI - angle;
}

Eigen::Vector3d quaternionRatesToBodyRates(const Eigen::Quaterniond& q2,
                                           const Eigen::Quaterniond& q1,
                                           const double dt,
                                           const bool rates_in_body_frame) {
  // Computes the angular velocity in body coordinate system for a rotation
  // from q1 to q2 within the time dt

  Eigen::Quaterniond q_dot;

  // Note, that the space of unit quaternion S(3) double covers the space
  // of physical attitudes SO(3) therefore q = -q.
  // I want the minimal q_dot, therefore I need to check which is
  // better q or -q (they both represent the same attitude in space)

  Eigen::Quaterniond q2_best = q2;
  if ((-q2_best.coeffs() - q1.coeffs()).norm() <
      (q2_best.coeffs() - q1.coeffs()).norm()) {
    q2_best.coeffs() = -q2_best.coeffs();
  }
  q_dot.coeffs() = (q2_best.coeffs() - q1.coeffs()) / dt;

  // [ o W_omega ]' = 2q_dot * q_bar in world frame
  // [ o B_omega ]' = 2q_bar * q_dot in body frame
  if (rates_in_body_frame) {
    return Eigen::Vector3d(2.0 * (q2_best.conjugate() * q_dot).vec());
  } else {
    return Eigen::Vector3d(2.0 * (q_dot * q2_best.conjugate()).vec());
  }
}

Eigen::Vector3d quaternionDeltaToBodyRates(const Eigen::Quaterniond& q2,
                                           const Eigen::Quaterniond& q1,
                                           const double dt,
                                           const bool rates_in_body_frame) {
  // Computes the angular velocity in body coordinate system for a rotation
  // from q1 to q2 within the time dt
  // This is basically the inverse of the integrateQuaternion function

  Eigen::Quaterniond q_e = q1.inverse() * q2;

  if (q_e.w() < 0) {
    // Make sure real part of quaternion has positive sign such that we take
    // the minimum distance from q1 to q2 to compute the body rates
    q_e = Eigen::Quaterniond(-q_e.w(), -q_e.x(), -q_e.y(), -q_e.z());
  }

  double a = sinc(acos(q_e.w()) / M_PI);

  Eigen::Vector3d alpha(q_e.x() / a, q_e.y() / a, q_e.z() / a);

  Eigen::Vector3d omega = 2.0 * alpha / dt;

  if (rates_in_body_frame) {
    return omega;
  } else {
    return q2 * omega;
  }
}

double angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return wrapMinusPiToPi(std::acos(a.dot(b)));
}

Eigen::Vector3d angleAxisFromTwoNormals(const Eigen::Vector3d& a,
                                        const Eigen::Vector3d& b,
                                        const Eigen::Vector3d& a_perp) {
  // FromTwoVectors()                             
  Eigen::Vector3d rvec;
  const Eigen::Vector3d cross = a.cross(b);
  const double crossNorm = cross.norm();
  const double c = a.dot(b);
  const double angle = std::acos(c);
  if (crossNorm < 1e-6) {
    if (c > 0) {
      rvec = cross;
    } else {
      rvec = a_perp * M_PI;
    }
  } else {
    rvec = cross * (angle / crossNorm);
  }
  return rvec;
}

double getHeading(Eigen::Ref<Eigen::Vector3d> acc, Eigen::Ref<Eigen::Vector3d> vel,
                  Eigen::Quaterniond& lastTilt, double& lastHeading) {
  double heading{0.0};
  const Eigen::Vector3d thrust_vec = acc - Eigen::Vector3d(0.0, 0.0, -9.8066);
  const double thrust = thrust_vec.norm();
  const Eigen::Quaterniond q_pitch_roll =
    thrust > 1e-3 ? Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), thrust_vec)
                  : lastTilt;
  lastTilt = q_pitch_roll;

  const Eigen::Vector3d v_body = q_pitch_roll.inverse() * vel;
  if ((v_body.x() * v_body.x() + v_body.y() * v_body.y()) > 1e-6) {
    heading = std::atan2(v_body.y(), v_body.x());
  } else {
    heading = lastHeading;
  }
  return heading;                 
}     


// Vector<> clip(const Vector<>& v, const Vector<>& bound) {
//   return v.cwiseMin(bound).cwiseMax(-bound);
// }
// Matrix<4, 4> Q_left(const Quaternion& q) {
//   return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(),
//           q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(), q.x(), q.w())
//     .finished();
// }

// Matrix<4, 4> Q_right(const Quaternion& q) {
//   return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(),
//           -q.y(), q.y(), -q.z(), q.w(), q.x(), q.z(), q.y(), -q.x(), q.w())
//     .finished();
// }
}  // namespace drolib

// template <typename Derived>
// Eigen::Quaternion<typename Derived::Scalar> quatDelta(
//     const Eigen::MatrixBase<Derived> &theta) {
//   using Scalar_t = typename Derived::Scalar;

//   Eigen::Quaternion<Scalar_t> dq;
//   Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
//   half_theta /= static_cast<Scalar_t>(2.0);
//   dq.w() = static_cast<Scalar_t>(1.0);
//   dq.x() = half_theta.x();
//   dq.y() = half_theta.y();
//   dq.z() = half_theta.z();
//   return dq;
// }