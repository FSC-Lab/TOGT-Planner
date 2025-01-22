#include "drolib/race/race_planner.hpp"
#include <chrono>

namespace drolib {

RacePlanner::RacePlanner(const RaceParams &params, double sampleTime)
    : params_(params), trajSampleTimeSec_(sampleTime) {
  quad_ = QuadManifold(params.qp);  
}

bool RacePlanner::solve(std::shared_ptr<RaceTrack> track,
                        const TrajParams &tparams, const LbfgsParams &lbfgs) {
  ready_ = solver_.solve(track->initState.toPVAJ(), track->endState.toPVAJ(),
                       quad_, tparams, lbfgs);

  return ready_;       
}

bool RacePlanner::planAOS(std::shared_ptr<RaceTrack> track) {
  if (params_.tprefine.piecesPerSegment == 1) {
    return planTOGT(track);
  }

  // Solve initial guesses
  {
    bool success{false};
    const int max_trials = 10;
    double speedGuess = params_.tpinit.speedGuess;
    for(int i = 0; i < max_trials; ++i) {
      TrajData newdata;
      track->initCorridors(params_.tpinit.piecesPerSegment - 1);
      track->getData(speedGuess, newdata);
      solver_.setInitialGuess(newdata);
      solver_.setConstYawBeforeTilt();
      if (solve(track, params_.tpinit, params_.lpinit)) {
        success = true;
        break;
      } else{
        speedGuess += i;
        std::cout << "Increase speedGuess to " << speedGuess << "\n";
      }
    }

    if (!success) {
      std::cout << "First optimizatoin fails!\n";
      return false;
    }
  }
  TrajData initdata = solver_.data; 

  {
    bool success{false};
    int piecesPerSegment = params_.tprefine.piecesPerSegment;
    for(int i = piecesPerSegment; i > 4;--i) {
      TrajData refinedata;
      track->initCorridors(piecesPerSegment - 1);
      track->getData(initdata, refinedata);

      solver_.setInitialGuess(refinedata);
      if (solve(track, params_.tprefine, params_.lprefine)) {
        success = true;
        break;
      } else {
        piecesPerSegment--;
        std::cout << "Decrease piecesPerSegment to " << piecesPerSegment << "\n";
      }
    }

    if (!success) {
      std::cout << "Second optimizatoin fails!\n";
      if (forwardHeading_) {
        std::cout << "Drone will keep facing forward!" << std::endl;
        Eigen::Vector3d diff = initdata.traj.getPos(trajSampleTimeSec_) -
                              initdata.traj.getPos(0.0);
        const double initHeading = std::atan2(diff.y(), diff.x());
        trajectory_ =
            MincoSnapTrajectory(params_.qp.name, quad_, initdata, initHeading,
                                track->getName() + " Trajectory");
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_, true);
      } else {
        std::cout << "Drone will keep maintaining zero yaw angle!" << std::endl;
        htype_ = MincoSnapTrajectory::HeadingType::CONSTANT_HEADING;
        trajectory_ = MincoSnapTrajectory(
            params_.qp.name, quad_, initdata, desiredYaw_, desiredYaw_,
            track->getName() + " Trajectory", rtype_, htype_);
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
      }
      std::cout << "Use first optimizatoin result!\n";
      return true;
      // return false;
    }

  }

  if (forwardHeading_) {
    Eigen::Vector3d diff = solver_.data.traj.getPos(trajSampleTimeSec_) -
                          solver_.data.traj.getPos(0.0);
    const double initHeading = std::atan2(diff.y(), diff.x());
    trajectory_ =
        MincoSnapTrajectory(params_.qp.name, quad_, solver_.data, initHeading,
                            track->getName() + " Trajectory");
    extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_, true);
  } else {
    htype_ = MincoSnapTrajectory::HeadingType::CONSTANT_HEADING;
    trajectory_ = MincoSnapTrajectory(
        params_.qp.name, quad_, solver_.data, desiredYaw_, desiredYaw_,
        track->getName() + " Trajectory", rtype_, htype_);
    extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
  }

  track->updateWaypoints(solver_.data);
  return true;
}

bool RacePlanner::plan(std::shared_ptr<RaceTrack> track) {
  // forwardHeading_ = true;

  std::cout << "Forward is " << forwardHeading_ << std::endl;

  if (params_.tprefine.piecesPerSegment == 1) {
    return planTOGT(track);
  }

  // Solve initial guesses
  {
    auto start_time_point_ = std::chrono::high_resolution_clock::now();

    TrajData newdata;
    track->initCorridors(params_.tpinit.piecesPerSegment - 1);

    track->getData(params_.tpinit.speedGuess, newdata);
    solver_.setInitialGuess(newdata);
    solver_.setConstYawBeforeTilt();
    if (!solve(track, params_.tpinit, params_.lpinit)) {
      std::cout << "First optimizatoin fails!\n";
      return false;
    }

    auto end_timepoint = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(
                     start_time_point_)
                     .time_since_epoch()
                     .count();
    auto end =
        std::chrono::time_point_cast<std::chrono::microseconds>(end_timepoint)
            .time_since_epoch()
            .count();
    auto duration = end - start;
    const double kMilliSecToSec = 0.001;
    double ms = static_cast<double>(duration) * kMilliSecToSec;

    std::cout << "Time for initialization:" << duration << " us (" << ms << " ms)\n";
  }

 // Refine the trajectory
  TrajData initdata, refinedata; 
  {
    // track->segments.clear();

    auto start_time_point_ = std::chrono::high_resolution_clock::now();

    // TrajData refinedata;
    track->initCorridors(params_.tprefine.piecesPerSegment - 1);
    track->getData(solver_.data, refinedata);

    initdata = solver_.data;
    solver_.setInitialGuess(refinedata);
    // TODO(chao): fix this bug of using previous segments to extract initial guesses
    track->resetSegments();

    if (!solve(track, params_.tprefine, params_.lprefine)) {
      std::cout << "Second optimizatoin fails!\n";
      if (forwardHeading_) {
        std::cout << "Using forward heading" << std::endl;
        Eigen::Vector3d diff = initdata.traj.getPos(trajSampleTimeSec_) -
                              initdata.traj.getPos(0.0);
        const double initHeading = std::atan2(diff.y(), diff.x());
        trajectory_ =
            MincoSnapTrajectory(params_.qp.name, quad_, initdata, initHeading,
                                track->getName() + " Trajectory");
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_, true);
      } else {
        trajectory_ = MincoSnapTrajectory(
            params_.qp.name, quad_, initdata, desiredYaw_, desiredYaw_,
            track->getName() + " Trajectory", rtype_, htype_);
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
      }
      std::cout << "Use first optimizatoin result!\n";
      // return false;
    } else {
      if (forwardHeading_) {
        std::cout << "Using forward heading" << std::endl;

        Eigen::Vector3d diff = solver_.data.traj.getPos(trajSampleTimeSec_) -
                              solver_.data.traj.getPos(0.0);
        const double initHeading = std::atan2(diff.y(), diff.x());
        trajectory_ =
            MincoSnapTrajectory(params_.qp.name, quad_, solver_.data, initHeading,
                                track->getName() + " Trajectory");
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_, forwardHeading_);
        trajectory_.getSetpointVecByDist(sampleDistMeter_);

      } else {
        std::cout << "Not using forward heading" << std::endl;

        trajectory_ = MincoSnapTrajectory(
            params_.qp.name, quad_, solver_.data, desiredYaw_, desiredYaw_,
            track->getName() + " Trajectory", rtype_, htype_);
        extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
        trajectory_.getSetpointVecByDist(sampleDistMeter_);
      }
    }   

    auto end_timepoint = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(
                     start_time_point_)
                     .time_since_epoch()
                     .count();
    auto end =
        std::chrono::time_point_cast<std::chrono::microseconds>(end_timepoint)
            .time_since_epoch()
            .count();
    auto duration = end - start;
    const double kMilliSecToSec = 0.001;
    double ms = static_cast<double>(duration) * kMilliSecToSec;

    std::cout << "Time for AOS:" << duration << " us (" << ms << " ms)\n";

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
    extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_, true);
  } else {
    htype_ = MincoSnapTrajectory::HeadingType::CONSTANT_HEADING;
    trajectory_ = MincoSnapTrajectory(
        params_.qp.name, quad_, solver_.data, desiredYaw_, desiredYaw_,
        track->getName() + " Trajectory", rtype_, htype_);
    extremum_ = trajectory_.getSetpointVec(trajSampleTimeSec_);
  }
 
  track->updateWaypoints(solver_.data);
  return true;
}

} // namespace drolib