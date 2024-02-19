
#include "drolib/math/math_utils.h"

namespace drolib {

double deg2rad(const double deg) { return deg * 0.017453292519943; }

double rad2deg(const double rad) { return rad * 57.295779513082323; }

double sinc(const double a) {
  if (fabs(a) >= 1e-5) {
    return sin(M_PI * a) / (M_PI * a);
  } else {
    return 1.0;
  }
}

double wrapZeroToTwoPi(const double angle) {
  if (angle >= 0.0 && angle <= 2.0 * M_PIl) {
    return angle;
  }
  double wrapped_angle = fmod(angle, 2.0 * M_PIl);
  if (wrapped_angle < 0.0) {
    wrapped_angle += 2.0 * M_PIl;
  }
  return wrapped_angle;
}

double wrapMinusPiToPi(const double angle) {
  if (angle >= -M_PIl && angle <= M_PIl) {
    return angle;
  }
  double wrapped_angle = angle + M_PIl;
  wrapped_angle = wrapZeroToTwoPi(wrapped_angle);
  wrapped_angle -= M_PIl;
  return wrapped_angle;
}

double wrapAngleDifference(const double current_angle,
                           const double desired_angle) {
  double angle_diff =
      wrapZeroToTwoPi(desired_angle) - wrapZeroToTwoPi(current_angle);
  if (angle_diff > M_PIl) {
    angle_diff = (-2.0 * M_PIl + angle_diff);
  }
  if (angle_diff < -M_PIl) {
    angle_diff = (2.0 * M_PIl + angle_diff);
  }
  return angle_diff;
}

Eigen::VectorXd clip(const Eigen::VectorXd& v, const Eigen::VectorXd& bound) {
  return v.cwiseMin(bound).cwiseMax(-bound);
}

Eigen::Matrix3d pseudoInverse(const Eigen::Matrix3d &mat) {
  // https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double epsilon = std::numeric_limits<double>::epsilon();
  // For a non-square matrix
  // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU |
  // Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(mat.cols(), mat.rows()) *
                     svd.singularValues().array().abs()(0);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tolerance)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
  return v.head<2>() / v[2];
}

Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
  return Eigen::Vector3d(v[0], v[1], 1.0);
}

}  // namespace drolib
