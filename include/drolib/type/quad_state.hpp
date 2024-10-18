#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "drolib/rotation/rotation_utils.h"
#include "drolib/type/types.hpp"
#include "drolib/base/parameter_base.hpp"

namespace drolib {

class QuadState : public ParameterBase {
 public:

  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    OME = 10,
    OMEX = 10,
    OMEY = 11,
    OMEZ = 12,
    NOME = 3,
    ACC = 13,
    ACCX = 13,
    ACCY = 14,
    ACCZ = 15,
    NACC = 3,
    TAU = 16,
    TAUX = 16,
    TAUY = 17,
    TAUZ = 18,
    NTAU = 3,
    JERK = 19,
    JERKX = 19,
    JERKY = 20,
    JERKZ = 21,
    NJERK = 3,
    SNAP = 22,
    SNAPX = 22,
    SNAPY = 23,
    SNAPZ = 24,
    NSNAP = 3,
    SIZE = 25
  };

  using StateVector = Eigen::Matrix<double, IDX::SIZE, 1>;

  QuadState();
  QuadState(const double t, const StateVector& x);
  QuadState(const double t, const Eigen::Vector3d& position, const double yaw);
  QuadState(const QuadState& state);
  ~QuadState();

  using ParameterBase::load;
  bool load(const Yaml& yaml) override;

  
  inline bool valid() const override { return x.allFinite() && std::isfinite(t); }

  inline PVAJ toPVAJ() const {
    return (Eigen::Matrix<double, 3, 4>() << p, v, a, j).finished();
  }

  void write(const std::string& name, std::ofstream& os);

  static inline int size() { return SIZE; }
  Eigen::Quaterniond q() const;
  void q(const Eigen::Quaterniond& quaternion);
  void q(const double angle,
         const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  void q(const Eigen::Vector3d& rpy);

  Eigen::Matrix3d R() const;

  void setZero(const bool& reset_time = true);


  StateVector x = StateVector::Constant(NAN);
  double t{NAN};

  Eigen::Ref<Eigen::Vector3d> p{x.segment<IDX::NPOS>(IDX::POS)};
  Eigen::Ref<Eigen::Vector4d> qx{x.segment<IDX::NATT>(IDX::ATT)};
  Eigen::Ref<Eigen::Vector3d> v{x.segment<IDX::NVEL>(IDX::VEL)};
  Eigen::Ref<Eigen::Vector3d> w{x.segment<IDX::NOME>(IDX::OME)};
  Eigen::Ref<Eigen::Vector3d> a{x.segment<IDX::NACC>(IDX::ACC)};
  Eigen::Ref<Eigen::Vector3d> tau{x.segment<IDX::NTAU>(IDX::TAU)};
  Eigen::Ref<Eigen::Vector3d> j{x.segment<IDX::NJERK>(IDX::JERK)};
  Eigen::Ref<Eigen::Vector3d> s{x.segment<IDX::NSNAP>(IDX::SNAP)};

  double getYaw(const double yaw = 0.0) const;
  
  inline double getTiltedAngle() const {
    return std::acos(1.0 - 2.0 * (qx(1) * qx(1) + qx(2) * qx(2)));
  }

  double getYawBeforeTilt() const {
    Eigen::Vector3d zB = q() * Eigen::Vector3d::UnitZ();
    return getYawBeforeTilt(zB);
  }

  double getYawBeforeTilt(const Eigen::Vector3d &bearing) const;

  // void applyYaw(const double angle);
  QuadState getHoverState() const;
  void normalizeQ();

  bool operator==(const QuadState& rhs) const;
  QuadState& operator=(const QuadState& other);
  bool isApprox(const QuadState& rhs, const double tol = 1e-6) const;

  inline bool operator<(const double time) const { return t < time; }
  inline bool operator<=(const double time) const { return t <= time; }
  inline bool operator>(const double time) const { return t > time; }
  inline bool operator>=(const double time) const { return t >= time; }

  friend std::ostream& operator<<(std::ostream& os, const QuadState& state);
};

}  // namespace drolib