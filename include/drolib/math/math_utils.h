#pragma once

#include <Eigen/Dense>

namespace drolib {

double deg2rad(const double deg);
double rad2deg(const double rad);
double sinc(const double a);
double wrapZeroToTwoPi(const double angle);
double wrapMinusPiToPi(const double angle);
double wrapAngleDifference(const double current_angle,
                           const double desired_angle);
Eigen::VectorXd clip(const Eigen::VectorXd& v, const Eigen::VectorXd& bound);

Eigen::Matrix3d pseudoInverse(const Eigen::Matrix3d &mat);
Eigen::Vector2d project2d(const Eigen::Vector3d& v);
Eigen::Vector3d unproject2d(const Eigen::Vector2d& v);

}  // namespace drolib
