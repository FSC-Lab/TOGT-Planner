#include "drolib/type/quad_state.hpp"

namespace drolib {

QuadState::QuadState() {}

QuadState::QuadState(const double t, const StateVector& x) : x(x), t(t) {}

QuadState::QuadState(const double t, const Eigen::Vector3d& position,
                     const double yaw)
    : t(t) {
  x.setZero();
  p = position;
  q(yaw);
}

QuadState::QuadState(const QuadState& state) : x(state.x), t(state.t) {}

QuadState::~QuadState() {}

// bool QuadState::load(const Yaml& yaml) {
//   bool got{true};
//   got &= yaml["pos"].getIfDefined(p);
//   got &= yaml["vel"].getIfDefined(v);
//   got &= yaml["acc"].getIfDefined(a);
//   got &= yaml["jer"].getIfDefined(j);
//   got &= yaml["rot"].getIfDefined(qx);
//   return got;
// }

void QuadState::write(const std::string &name, std::ofstream& os) {
  const Eigen::Vector3d rpy = quaternionToEulerAnglesRPY(q());
  const double cthrustmass = (a + Eigen::Vector3d(0.0, 0.0, 9.8066)).norm();
  os.precision(5);
  os << name << ":\n"
      << "  pos: [" << p.x() << ", " << p.y() << ", " << p.z() << "]\n"
      << "  vel: [" << v.x() << ", " << v.y() << ", " << v.z() << "]\n"
      << "  cthrustmass: " << cthrustmass << "\n"
      << "  euler: [" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "]\n\n";
  os.precision();
}

bool QuadState::load(const Yaml& yaml) {
  if (yaml.isNull()) return false;

  setZero();
  bool got{true};
  got &= yaml["pos"].getIfDefined(p);
  got &= yaml["vel"].getIfDefined(v);

  Eigen::Vector3d rpy;
  double cthrustmass{0.0};
  got &= yaml["cthrustmass"].getIfDefined(cthrustmass);
  got &= yaml["euler"].getIfDefined(rpy);

  q(deg2rad(rpy));
  a = q() * Eigen::Vector3d(0.0, 0.0, cthrustmass) - Eigen::Vector3d(0.0, 0.0, 9.8066);
  j.setZero();

  return got;
}

Eigen::Quaterniond QuadState::q() const {
  return Eigen::Quaterniond(x(ATTW), x(ATTX), x(ATTY), x(ATTZ));
}

void QuadState::q(const Eigen::Quaterniond& quaternion) {
  x(IDX::ATTW) = quaternion.w();
  x(IDX::ATTX) = quaternion.x();
  x(IDX::ATTY) = quaternion.y();
  x(IDX::ATTZ) = quaternion.z();
}

void QuadState::q(const double angle, const Eigen::Vector3d& axis) {
  const Eigen::Quaterniond q_aa(Eigen::AngleAxis<double>(angle, axis));
  x(IDX::ATTW) = q_aa.w();
  x(IDX::ATTX) = q_aa.x();
  x(IDX::ATTY) = q_aa.y();
  x(IDX::ATTZ) = q_aa.z();
}

void QuadState::q(const Eigen::Vector3d& rpy) {
  q(eulerAnglesRPYToQuaternion(rpy));
}

Eigen::Matrix3d QuadState::R() const {
  return Eigen::Quaterniond(x(ATTW), x(ATTX), x(ATTY), x(ATTZ))
      .toRotationMatrix();
}

void QuadState::setZero(const bool& reset_time) {
  if (reset_time) {
    t = 0.0;
  }
  x.setZero();
  x(ATTW) = 1.0;
}

double QuadState::getYaw(const double yaw) const { return quaternionToYaw(q(), yaw); }

double QuadState::getYawBeforeTilt(const Eigen::Vector3d &bearing) const {
  Eigen::Quaterniond qz = quaternionFromUnitZToV(bearing).inverse() * q();
  return quaternionToYaw(qz, 0.0);
}

QuadState QuadState::getHoverState() const {
  QuadState hover_state;
  hover_state.setZero();
  hover_state.t = t;
  hover_state.p = p;
  hover_state.q(getYaw(0.0));
  return hover_state;
}

bool QuadState::operator==(const QuadState& rhs) const {
  return t == rhs.t && x.isApprox(rhs.x);
}

bool QuadState::isApprox(const QuadState& rhs, const double tol) const {
  return std::abs(t - rhs.t) < tol && x.isApprox(rhs.x, tol);
}

void QuadState::normalizeQ() {
  const double q_norm = q().norm();
  qx(0) = qx(0) / q_norm;
  qx(1) = qx(1) / q_norm;
  qx(2) = qx(2) / q_norm;
  qx(3) = qx(3) / q_norm;
}

std::ostream& operator<<(std::ostream& os, const QuadState& state) {
  os.precision(6);
  // os << std::scientific;
  os << "State at " << state.t << "s:\n";
  os.precision(3);
  os << "p=  [" << state.p.transpose() << "]\n"
     << "q=  [" << state.qx.transpose() << "]\n"
     << "v=  [" << state.v.transpose() << "]\n"
     << "w=  [" << state.w.transpose() << "]\n"
     << "a=  [" << state.a.transpose() << "]\n"
     << "j=  [" << state.j.transpose() << "]\n"
     << "s=  [" << state.s.transpose() << "]\n"
     << "tau=[" << state.tau.transpose() << "]" << std::endl;
  os.precision();
  // os.unsetf(std::ios::scientific);
  return os;
}

}  // namespace drolib