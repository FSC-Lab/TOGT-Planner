#pragma once

#include "drolib/planner/angle.hpp"
#include "drolib/planner/traj_data.hpp"
#include "drolib/rotation/rotation_utils.h"
#include "drolib/type/types.hpp"

#include "drolib/type/set_point.hpp"
#include "drolib/system/quadrotor_manifold.hpp"
#include "drolib/math/min_max_recorder.hpp"
// #include <numeric>
namespace drolib {

struct TrajExtremum {
  double maxTime{NAN};
  double length{NAN};
  MinMaxRecorder<double> vel;
  MinMaxRecorder<double> acc;
  MinMaxRecorder<double> tilt;
  MinMaxRecorder<double> collectiveThrust;
  MinMaxVecRecorder<double, 3> omg;
  MinMaxVecRecorder<double, 3> rpy;
  MinMaxVecRecorder<double, 4> thrusts;

  inline void reset() {
    maxTime = NAN;
    vel.reset();
    acc.reset();
    tilt.reset();
    collectiveThrust.reset();
    omg.reset();
    rpy.reset();
    thrusts.reset();
  }

  friend std::ostream &operator<<(std::ostream &os,
                                  const TrajExtremum &extremum) {
    os.precision(4);
    // os << std::scientific;
    os << std::fixed;
    os << "TrajExtremum:\n"
      << "maxVel:     " << extremum.vel.max() << " [m/s]\n"
      << "maxAcc:     " << extremum.acc.max() << " [m^2/s]\n"
      << "maxTilt:    " << rad2deg(extremum.tilt.max()) << " [deg]\n"
      << "minOmg:     " << extremum.omg.min().transpose() << " [rad/s]\n"
      << "maxOmg:     " << extremum.omg.max().transpose() << " [rad/s]\n"
      << "minEuler:   " << extremum.rpy.min().transpose() << " [deg]\n"
      << "maxEuler:   " << extremum.rpy.max().transpose() << " [deg]\n";
    if (extremum.thrusts.min().norm() < 1.0e6 &&
        extremum.thrusts.max().norm() < 1.0e6) {
      os << "minThrusts: " << extremum.thrusts.min().transpose() << " [N]\n"
        << "maxThrusts: " << extremum.thrusts.max().transpose() << " [N]\n"
        << "minCthrust: " << extremum.thrusts.min().sum() << " [N]\n"
        << "maxCthrust: " << extremum.thrusts.max().sum() << " [N]\n";
    } else {
      os << "minCthrust: " << extremum.collectiveThrust.min() << " [N]\n"
        << "maxCthrust: " << extremum.collectiveThrust.max() << " [N]\n";
    }
    os << "-----------------------------------------\n"
       << "Length:   " << extremum.length << " [m]\n"
       << "Duration:   " << extremum.maxTime << " [s]" << std::endl;
    os.precision();
    // os.unsetf(std::ios::scientific);
    return os;
  }

};

struct MincoSnapTrajectory {
  enum RotationType : uint8_t { TILT_HEADING = 0, ROLL_PITCH_YAW = 1 };
  enum HeadingType : uint8_t {
    CONSTANT_HEADING = 0,
    SMOOTH_HEADING = 1,
    FORWARD_HEADING = 2
  };

  MincoSnapTrajectory(const std::string quad_name, 
                      const QuadManifold& quad,
                      const TrajData &data,
                      const double start_yaw, const double end_yaw = 0.0,
                      const std::string& name = "MincoSnap Trajectory",
                      const RotationType rtype = RotationType::TILT_HEADING,
                      const HeadingType htype = HeadingType::CONSTANT_HEADING);

  MincoSnapTrajectory(const std::string quad_name, 
                      const QuadManifold& quad,
                      const TrajData &data,
                      const double start_yaw,
                      const std::string& name = "MincoSnap Trajectory");

  MincoSnapTrajectory() = default;

  ~MincoSnapTrajectory() = default;

  inline bool valid() const {
    bool check{true};
    check &= start_pvaj.allFinite();
    check &= end_pvaj.allFinite();
    check &= waypoints.cols() == durations.size() - 1;
    check &= durations.size() > 0;
    check &= std::isfinite(start_yaw);
    check &= std::isfinite(end_yaw);
    return check;
  }

  bool save(const std::string &filename);

  bool saveSegments(const std::string &filename, const int piecesPerSegment);

  bool saveAllWaypoints(const std::string &filename);

  inline double getTotalDuration() const { return polys.getTotalDuration(); }

  TrajExtremum getSetpointVec(const double sampleTimeSecond = 0.01); 

  std::string name;
  std::string quad_name;
  QuadManifold quad;
  PiecewisePolynomial<POLY_DEG> polys;
  PVAJ start_pvaj;
  PVAJ end_pvaj;
  Eigen::Matrix3Xd waypoints;
  Eigen::VectorXd durations;
  double start_yaw{0.0};
  double end_yaw{0.0};
  RotationType rotation_type{TILT_HEADING};
  HeadingType heading_type{CONSTANT_HEADING};

  SetpointVector setpoints;

  friend std::ostream &operator<<(std::ostream &os, const MincoSnapTrajectory &traj);
};

} // namespace drolib