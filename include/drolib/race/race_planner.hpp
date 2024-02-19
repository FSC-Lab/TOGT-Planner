#pragma once

#include "drolib/math/min_max_recorder.hpp"
#include "drolib/planner/traj_solver.hpp"
#include "drolib/race/race_track.hpp"
#include "drolib/race/race_params.hpp"
#include "drolib/system/minco_snap_trajectory.hpp"
#include "drolib/system/quadrotor_manifold.hpp"
#include "drolib/type/quad_state.hpp"
#include "drolib/type/set_point.hpp"
#include "drolib/utils/logger.hpp"
#include "drolib/utils/timer.hpp"

namespace drolib {

class RacePlanner {
public:
  RacePlanner() = default;
  RacePlanner(const RaceParams &parmas, double sampleTime = 0.01);
  ~RacePlanner() {}
  bool solve(std::shared_ptr<RaceTrack> track, const TrajParams &tparams,
             const LbfgsParams &lbfgs);
  bool plan(std::shared_ptr<RaceTrack> track);
  bool planTOGT(std::shared_ptr<RaceTrack> track);
  MincoSnapTrajectory getTrajectory(void) { return trajectory_; }
  TrajExtremum getExtremum(void) { return extremum_; }
  TrajData getSolution(void) { return solver_.data; }

private:
  Timer timerPlanning_{"Planning"};
  Logger logger_{"RacePlanner"};

  RaceParams params_;

  QuadManifold quad_;
  TrajSolver solver_;

  double trajSampleTimeSec_{0.01};
  MincoSnapTrajectory trajectory_;
  TrajExtremum extremum_;

  bool forwardHeading_{true};
  double desiredYaw_{0.0};
  MincoSnapTrajectory::RotationType rtype_{
      MincoSnapTrajectory::RotationType::TILT_HEADING};
  MincoSnapTrajectory::HeadingType htype_{
      MincoSnapTrajectory::HeadingType::CONSTANT_HEADING};
};

} // namespace drolib