#pragma once

#include <Eigen/Dense>
#include "drolib/math/math_utils.h"

namespace drolib {

Eigen::Vector3d rad2deg(const Eigen::Vector3d& rad);
Eigen::Vector3d deg2rad(const Eigen::Vector3d& deg);
Eigen::Quaterniond quatDelta(const Eigen::Vector3d& theta);
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Vector3d quaternionToEulerAnglesRPY(const Eigen::Quaterniond& q);
Eigen::Vector3d quaternionToAngleAxis(const Eigen::Quaterniond& q);
Eigen::Quaterniond eulerAnglesRPYToQuaternion(const Eigen::Vector3d& rpy);
Eigen::Quaterniond angleAxisToQuaternion(const Eigen::Vector3d& rvec);
Eigen::Vector3d rotationMatrixToEulerAnglesRPY(const Eigen::Matrix3d& R);
Eigen::Matrix3d eulerAnglesRPYToRotationMatrix(const Eigen::Vector3d& rpy);
Eigen::Quaterniond quaternionAtUnitX(const double roll);
Eigen::Quaterniond quaternionAtUnitY(const double pitch);
Eigen::Quaterniond quaternionAtUnitZ(const double yaw);
Eigen::Quaterniond quaternionFromUnitZToV(const Eigen::Vector3d &v);
double quaternionToYaw(const Eigen::Quaterniond &q, const double yaw = NAN);

Eigen::Vector3d quaternionRatesToBodyRates(
    const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q1, const double dt,
    const bool rates_in_body_frame = true);
Eigen::Vector3d quaternionDeltaToBodyRates(
    const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q1, const double dt,
    const bool rates_in_body_frame = true);
double angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

Eigen::Vector3d angleAxisFromTwoNormals(const Eigen::Vector3d& a,
                                        const Eigen::Vector3d& b,
                                        const Eigen::Vector3d& a_perp);

double getHeading(Eigen::Ref<Eigen::Vector3d> acc, Eigen::Ref<Eigen::Vector3d> vel,
                  Eigen::Quaterniond& lastTilt, double& lastHeading);               

// Vector<> clip(const Vector<>& v, const Vector<>& bound);
// Matrix<4, 4> Q_left(const Quaternion& q);
// Matrix<4, 4> Q_right(const Quaternion& q);
// Matrix<4, 3> qFromQeJacobian(const Quaternion& q);
                                      
}  // namespace drolib

// template <typename Derived>
// Eigen::Quaternion<typename Derived::Scalar> quatDelta(
//     const Eigen::MatrixBase<Derived> &theta);
