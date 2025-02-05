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
  
  TrajExtremum getSetpointVec(const double sampleTimeSecond = 0.01, const bool forward_heading = false); 

  std::string name;
  std::string quad_name;
  QuadManifold quad;
  double duration;
  PiecewisePolynomial<3> traj_xyz;
  PiecewisePolynomial<1> traj_yaw;
  PVAJ3D start_pvaj;
  PVAJ3D end_pvaj;
  double start_yaw;
  double end_yaw;
  SetpointVector setpoints;

};

}

#endif  /* DROLIB_SYSTEM_FLATOUTPUT_TRAJECTORY_HPP_ */
