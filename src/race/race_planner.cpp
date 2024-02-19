#include "drolib/race/race_planner.hpp"

namespace drolib {

RacePlanner::RacePlanner(const RaceParams &params, double sampleTime)
    : params_(params), trajSampleTimeSec_(sampleTime) {
  quad_ = QuadManifold(params.qp);  
}

bool RacePlanner::solve(std::shared_ptr<RaceTrack> track,
                        const TrajParams &tparams, const LbfgsParams &lbfgs) {
  return solver_.solve(track->initState.toPVAJ(), track->endState.toPVAJ(),
                       quad_, tparams, lbfgs);
}

bool RacePlanner::plan(std::shared_ptr<RaceTrack> track) {
  // Solve initial guesses
  {
    TrajData newdata;
    // std::cout << "piece num: " << params_.tpinit.piecesPerSegment - 1 << std::endl;
    track->initCorridors(params_.tpinit.piecesPerSegment - 1);
    track->getData(params_.tpinit.speedGuess, newdata);

    solver_.setInitialGuess(newdata);
    solver_.setConstYawBeforeTilt();
    if (!solve(track, params_.tpinit, params_.lpinit)) {
      std::cout << "First optimizatoin fails!\n";
      return false;
    }
  }

  // Refine the trajectory
  TrajData initdata, refinedata; 
  {
    // TrajData refinedata;
    track->initCorridors(params_.tprefine.piecesPerSegment - 1);
    track->getData(solver_.data, refinedata);
    initdata = solver_.data;
    solver_.setInitialGuess(refinedata);
    if (!solve(track, params_.tprefine, params_.lprefine)) {
      std::cout << "Second optimizatoin fails!\n";
      if (forwardHeading_) {
        Eigen::Vector3d diff = initdata.traj.getPos(trajSampleTimeSec_) -
                              initdata.traj.getPos(0.0);
        const double initHeading = std::atan2(diff.y(), diff.x());
        trajectory_ =
            MincoSnapTrajectory(params_.qp.name, quad_, initdata, initHeading,
                                track->getName() + " Trajectory");
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
      } else {
        trajectory_ = MincoSnapTrajectory(
            params_.qp.name, quad_, initdata, desiredYaw_, desiredYaw_,
            track->getName() + " Trajectory", rtype_, htype_);
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
      }
      std::cout << "Use first optimizatoin result!\n";
      return false;
    } else {
      if (forwardHeading_) {
        Eigen::Vector3d diff = solver_.data.traj.getPos(trajSampleTimeSec_) -
                              solver_.data.traj.getPos(0.0);
        const double initHeading = std::atan2(diff.y(), diff.x());
        trajectory_ =
            MincoSnapTrajectory(params_.qp.name, quad_, solver_.data, initHeading,
                                track->getName() + " Trajectory");
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
      } else {
        trajectory_ = MincoSnapTrajectory(
            params_.qp.name, quad_, solver_.data, desiredYaw_, desiredYaw_,
            track->getName() + " Trajectory", rtype_, htype_);
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
      }
    }    
  }
  
  track->updateWaypoints(solver_.data);
  return true;
}

bool RacePlanner::planTOGT(std::shared_ptr<RaceTrack> track) {

  TrajData newdata;
  track->initCorridors(0);
  track->getData(params_.tpinit.speedGuess, newdata);

  solver_.setInitialGuess(newdata);
  solver_.setConstYawBeforeTilt();
  if (!solve(track, params_.tpinit, params_.lpinit)) {
    std::cout << "Optimizatoin fails!\n";
    return false;
  }
  
  if (forwardHeading_) {
    Eigen::Vector3d diff = solver_.data.traj.getPos(trajSampleTimeSec_) -
                          solver_.data.traj.getPos(0.0);
    const double initHeading = std::atan2(diff.y(), diff.x());
    trajectory_ =
        MincoSnapTrajectory(params_.qp.name, quad_, solver_.data, initHeading,
                            track->getName() + " Trajectory");
    extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
  } else {
    trajectory_ = MincoSnapTrajectory(
        params_.qp.name, quad_, solver_.data, desiredYaw_, desiredYaw_,
        track->getName() + " Trajectory", rtype_, htype_);
    extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
  }
 
  track->updateWaypoints(solver_.data);
  return true;
}

} // namespace drolib