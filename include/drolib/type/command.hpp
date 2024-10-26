#pragma once

#include <Eigen/Eigen>
#include <cmath>

namespace drolib {

struct Command {

  Command();

  Command(const double t, const double thrust, const Eigen::Vector3d& omega);

  Command(const double t, const Eigen::Vector4d& thrusts);

  Command(const double t);

  bool valid() const;
  bool isSingleRotorThrusts() const;
  bool isRatesThrust() const;

  /// time in [s]
  double t{NAN};

  /// Collective mass-normalized thrust in [m/s^2]
  double collective_thrust{NAN};

  /// Bodyrates in [rad/s]
  Eigen::Vector3d omega{NAN, NAN, NAN};

  /// Single rotor thrusts in [N]
  Eigen::Vector4d thrusts{NAN, NAN, NAN, NAN};

  friend std::ostream& operator<<(std::ostream& os, const Command& command);

  bool operator==(const Command& rhs) const;
};

}  // namespace namespace drolib

