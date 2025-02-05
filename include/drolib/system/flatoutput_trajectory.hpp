#ifndef DROLIB_SYSTEM_FLATOUTPUT_TRAJECTORY_HPP_
#define DROLIB_SYSTEM_FLATOUTPUT_TRAJECTORY_HPP_

#include "drolib/planner/angle.hpp"
#include "drolib/planner/traj_se3_data.hpp"
#include "drolib/rotation/rotation_utils.h"
#include "drolib/type/types.hpp"
#include "drolib/polynomial/cubic_spline.h"

#include "drolib/type/set_point.hpp"
#include "drolib/system/quadrotor_manifold.hpp"
#include "drolib/math/min_max_recorder.hpp"
#include "minco_snap_trajectory.hpp"

namespace drolib {

struct FlatoutputTrajectory {
  FlatoutputTrajectory(const std::string quad_name, const QuadManifold& quad,
                      const TrajSE3Data &data);

  FlatoutputTrajectory() = default;

  ~FlatoutputTrajectory() = default;

  inline bool valid() const {
    bool check{true};
    check &= !setpoints.empty();
    return check;
  }

  TrajExtremum getSetpointVec(const double sampleTimeSecond = 0.01); 

  std::string name{"Flatoutput"};
  std::string quad_name;
  QuadManifold quad;
  double duration;
  PiecewisePolynomial<7, 3> traj_xyz;
  PiecewisePolynomial<7, 1> traj_yaw;
  PVAJ3D start_pvaj;
  PVAJ3D end_pvaj;
  PVAJ1D start_yaw;
  PVAJ1D end_yaw;
  SetpointVector setpoints;
  // Eigen::Matrix3Xd waypoints;
  // Eigen::VectorXd yaws;
  Eigen::VectorXd durations;
};

}

#endif  /* DROLIB_SYSTEM_FLATOUTPUT_TRAJECTORY_HPP_ */
