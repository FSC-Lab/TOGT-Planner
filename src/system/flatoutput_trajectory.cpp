#include "drolib/system/flatoutput_trajectory.hpp"

namespace drolib {

FlatoutputTrajectory::FlatoutputTrajectory(const std::string quad_name,
                                           const QuadManifold &quad,
                                           const TrajSE3Data &data)
    : quad_name(quad_name), quad(quad), traj_xyz(data.traj_xyz),
      traj_yaw(data.traj_yaw), durations(data.T), duration(data.T.sum()) {
  start_pvaj = traj_xyz.getPVAJ(0.0);
  end_pvaj = traj_xyz.getPVAJ(duration);
  start_yaw = traj_yaw.getPVAJ(0.0);
  end_yaw = traj_yaw.getPVAJ(duration);

  setpoints.clear();

}

TrajExtremum FlatoutputTrajectory::getSetpointVec(const double sampleTimeSecond) {
  TrajExtremum extremum;

  if (!quad.valid()) {
    return extremum;
  }

  const double T = duration;
  extremum.maxTime = T;
  const int nSamples = T / sampleTimeSecond;
  setpoints.clear();
  setpoints.reserve(nSamples);
 
  Setpoint setpoint;
  PVAJS3D xyz_pvajs;
  PVAJS1D yaw_pvaj;

  extremum.vel.add(0.0);
  extremum.vel.add(0.0);
  extremum.vel.add(traj_xyz.getMaxVel());
  extremum.acc.add(traj_xyz.getMaxAcc());

  double t{0.0};
  double dt{0.0};
  Eigen::Vector3d last_pos = traj_xyz.getPos(t);
  double length{0.0};
  for (int i = 0; i <= nSamples; ++i) {

    xyz_pvajs = traj_xyz.getPVAJS(t);
    yaw_pvaj = traj_yaw.getPVAJS(t);
    Eigen::Vector3d heading = yaw_pvaj.head<3>();
    // std::cout << "heading: " << heading.transpose() * 180.0 / M_PI << std::endl;

    Eigen::Vector3d pos = xyz_pvajs.col(0);
    length += (pos - last_pos).norm();
    last_pos = pos;

    quad.toStateWithTiltYaw(t, xyz_pvajs, heading, setpoint);
    extremum.thrusts.add(setpoint.input.thrusts);
    extremum.collectiveThrust.add(setpoint.input.collective_thrust);
    extremum.tilt.add(setpoint.state.getTiltedAngle());
    extremum.omg.add(setpoint.input.omega);
    extremum.rpy.add(rad2deg(quaternionToEulerAnglesRPY(setpoint.state.q())));
    Eigen::Vector3d rpy = rad2deg(quaternionToEulerAnglesRPY(setpoint.state.q()));
    std::cout << "yaw: " << rpy.z() << std::endl;

    setpoints.push_back(setpoint);

    dt = std::min(T - t, sampleTimeSecond);
    t += dt;
  }

  xyz_pvajs = traj_xyz.getPVAJS(t);
  yaw_pvaj = traj_yaw.getPVAJS(t);
  Eigen::Vector3d heading = yaw_pvaj.head<3>();
  quad.toStateWithTiltYaw(t, xyz_pvajs, heading, setpoint);
  setpoints.push_back(setpoint);

  Eigen::Vector3d pos = xyz_pvajs.col(0);
  length += (pos - last_pos).norm();

  extremum.length = length;

  return extremum;
}


} // namespace drolib