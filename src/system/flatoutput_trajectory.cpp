#include "drolib/system/flatoutput_trajectory.hpp"

namespace drolib {

FlatoutputTrajectory::FlatoutputTrajectory(const std::string quad_name,
                                           const QuadManifold &quad,
                                           const TrajSE3Data &data)
    : quad_name(quad_name), quad(quad), traj_xyz(data.traj_xyz),
      traj_yaw(data.traj_yaw), duration(data.T.sum()) {
  start_pvaj = traj_xyz.getPVAJ(0.0);
  end_pvaj = traj_xyz.getPVAJ(duration);
  start_yaw = traj_yaw.getPVAJ(0.0)(0);
}

} // namespace drolib