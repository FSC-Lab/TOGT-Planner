#include "drolib/system/minco_snap_trajectory.hpp"

namespace drolib {

MincoSnapTrajectory::MincoSnapTrajectory(
    const std::string quad_name, const QuadManifold &quad, const TrajData &data,
    const double start_yaw, const double end_yaw, const std::string &name,
    const RotationType rtype, const HeadingType htype)
    : name(name), quad_name(quad_name), quad(quad), polys(data.traj),
      start_pvaj(polys.getPVAJ(0.0)), end_pvaj(polys.getPVAJ(data.T.sum())),
      waypoints(data.P), durations(data.T), start_yaw(start_yaw),
      end_yaw(end_yaw), rotation_type(rtype), heading_type(htype) {}

MincoSnapTrajectory::MincoSnapTrajectory(const std::string quad_name,
                                         const QuadManifold &quad,
                                         const TrajData &data,
                                         const double start_yaw,
                                         const std::string &name)
    : name(name), quad_name(quad_name), quad(quad), polys(data.traj),
      start_pvaj(polys.getPVAJ(0.0)), end_pvaj(polys.getPVAJ(data.T.sum())),
      waypoints(data.P), durations(data.T), start_yaw(start_yaw),
      end_yaw(start_yaw), rotation_type(RotationType::TILT_HEADING),
      heading_type(HeadingType::CONSTANT_HEADING) {}

// TODO(chao): change here to modify the heading
TrajExtremum MincoSnapTrajectory::getSetpointVec(const double sampleTimeSec,
                                                 const bool forward_heading,
                                                 const bool tilt_convention) {
  TrajExtremum extremum;
  if (!quad.valid()) {
    return extremum;
  }

  const double T = polys.getTotalDuration();
  extremum.maxTime = T;

  const int nSamples = T / sampleTimeSec;
  setpoints.clear();
  setpoints.reserve(nSamples);

  Setpoint setpoint;
  PVAJS pvajs;
  Eigen::Vector3d yaw;

  double lastHeading = start_yaw;
  std::cout << "start yaw:" << start_yaw << std::endl;
  Eigen::Quaterniond lastTilt{1, 0, 0, 0};

  extremum.vel.add(0.0);
  extremum.vel.add(0.0);
  extremum.vel.add(polys.getMaxVel());
  extremum.acc.add(polys.getMaxAcc());

  double t{0.0};
  double dt{0.0};

  Eigen::Vector3d last_pos = polys.getPos(t);
  double length{0.0};
  // TODO: Set heading type manually here
  // if (forward_heading) {
  //   heading_type = HeadingType::FORWARD_HEADING;
  // } else {
  //   heading_type = HeadingType::CONSTANT_HEADING;
  // }

  for (int i = 0; i <= nSamples; ++i) {
    pvajs = polys.getPVAJS(t);

    Eigen::Vector3d pos = pvajs.col(0);
    length += (pos - last_pos).norm();
    last_pos = pos;

    // TODO: only support CONSTANT_HEADING and FORWARD_HEADING right now
    if (forward_heading) {
      // if (horizon < sampleTimeSec) {
      // yaw << wrapMinusPiToPi(getHeading(pvajs.col(2), pvajs.col(1), lastTilt, lastHeading)), 0.0, 0.0;

      // yaw << 0.0, 0.0, 0.0;
  
      // } else {
      double heading{0.0};
      const auto pvajs_future = polys.getPVAJS(std::min(T, t + horizon));
      const Eigen::Vector3d future_pos = pvajs_future.col(0);
      const Eigen::Vector3d diff = future_pos - pos;
      // std::cout << "diff: " << diff.transpose() << std::endl;
      if (diff.head<2>().norm() > 1e-3) {
        heading = wrapZeroToTwoPi(std::atan2(diff.y(), diff.x()));
      } else {
        heading = lastHeading;
      }
      yaw << heading, 0.0, 0.0;
      if (std::fabs(heading - lastHeading) > deg2rad(90)) {
        std::cout << "Yaw jump detected at " << t << ", current heading: " << heading << ", previous heading: " << lastHeading << std::endl;
      }
      lastHeading = heading;

      // }

      // std::cout << "FORWARD_HEADING: yaw " << yaw.transpose() << std::endl;
    } else {
      yaw << 0.0, 0.0, 0.0;
      //  std::cout << "NORMAL_HEADING: yaw " << yaw.transpose() << std::endl;
    }

    if (tilt_convention) {
      quad.toStateWithTiltYaw(t, pvajs, yaw, setpoint);
      Eigen::Quaterniond curr_quat = setpoint.state.q();
      if (t < 0.5) {
      std::cout << "yaw: " << yaw.transpose() << ", v: " << pvajs.col(1).transpose() << ", a: " << pvajs.col(2).transpose() << std::endl;

      }
      if (i == 0) {
        // do nothing
        if (curr_quat.w() < 0.0) {
          curr_quat.coeffs() = -curr_quat.coeffs();
          std::cout << "-------------Case 1-------------" << std::endl;
          // std::cout << "Flip detected at " << t << std::endl;
          // std::cout << "current quat_error: \n" << quat_error.norm() << std::endl;
          std::cout << "current q: " << curr_quat.coeffs().transpose()
                    << std::endl;
          // std::cout << "previous q: " << prev_quat.coeffs().transpose()
          //           << std::endl;
        }

      } else {

        double dot_product = prev_quat.coeffs().dot(curr_quat.coeffs());
        Eigen::Quaterniond quat_diff = curr_quat * prev_quat.inverse();
        Eigen::Vector3d quat_error =
            quatDiff(curr_quat.coeffs(), prev_quat.coeffs());
        double c_half_psi = cos(0.5 * yaw[0]);


        // else if (!quat_diff.coeffs()) {
        //         std::cout << "-------------Case 2-------------" << std::endl;
        //         std::cout << "Flip detected at " << t << std::endl;

        //       }

        // CHECK 1
        // if (curr_quat.w() < 0.0) {
        //   curr_quat.coeffs() = -curr_quat.coeffs();
        //   setpoint.state.q(curr_quat);
        //   std::cout << "-------------Case 1-------------" << std::endl;
        //   std::cout << "Flip detected at " << t << std::endl;
        //   std::cout << "current quat_error: \n" << quat_error.transpose() <<
        //   std::endl; std::cout << "current q: " <<
        //   curr_quat.coeffs().transpose() << std::endl; std::cout << "previous
        //   q: " << prev_quat.coeffs().transpose() << std::endl;

        // } else if (dot_product < 0.0) {
        //   setpoint.state.q(Eigen::Quaterniond(-curr_quat.w(), -curr_quat.x(),
        //                                       -curr_quat.y(),
        //                                       -curr_quat.z()));
        //   curr_quat = setpoint.state.q();
        //   std::cout << "-------------Case 3-------------" << std::endl;
        //   std::cout << "Flip detected at " << t << std::endl;
        //   std::cout << "current quat_error: \n" << quat_error.transpose() <<
        //   std::endl; std::cout << "current q: " <<
        //   curr_quat.coeffs().transpose() << std::endl; std::cout << "previous
        //   q: " << prev_quat.coeffs().transpose() << std::endl;
        // }

        // CHECK 2
        // if (dot_product < 0.0) {
        //   setpoint.state.q(Eigen::Quaterniond(curr_quat.w(), -curr_quat.x(),
        //                                       -curr_quat.y(),
        //                                       -curr_quat.z()));
        //   std::cout << "-------------Case 3-------------" << std::endl;
        //   std::cout << "Flip detected at " << t << std::endl;
        //   std::cout << "current quat_error: \n"
        //             << quat_error.norm() << std::endl;
        //   std::cout << "current q: " << curr_quat.coeffs().transpose()
        //             << std::endl;
        //   std::cout << "previous q: " << prev_quat.coeffs().transpose()
        //             << std::endl;
        //   curr_quat = setpoint.state.q();

        // }

        // CHECK 4
        // if ((prev_quat.coeffs() + curr_quat.coeffs()).squaredNorm() <
        //     (prev_quat.coeffs() - curr_quat.coeffs()).squaredNorm()) {
        //   // curr_quat.coeffs() = -curr_quat.coeffs();

        //   std::cout << "-------------Case 4-------------" << std::endl;
        //   std::cout << "Flip detected at " << t << std::endl;
        //   std::cout << "c_half_psi:" << c_half_psi << std::endl;
        //   std::cout << "current quat_error: \n"
        //             << quat_error.norm() << std::endl;
        //   std::cout << "current q: " << curr_quat.coeffs().transpose()
        //             << std::endl;
        //   std::cout << "previous q: " << prev_quat.coeffs().transpose()
        //             << std::endl;
        // }

        if (quat_error.norm() > 0.3) {
          Eigen::Vector3d pos = pvajs.col(0);
          Eigen::Vector3d vel = pvajs.col(1);  
          Eigen::Vector3d acc = pvajs.col(2);
          Eigen::Vector3d jer = pvajs.col(3);
          Eigen::Vector3d sna = pvajs.col(4);

          Eigen::Vector3d alpha = acc + Eigen::Vector3d(0, 0, G);
          Eigen::Vector3d z_B = alpha.normalized();


          std::cout << "-------------Large rotation jump-------------" << std::endl;
          std::cout << "Jump detected at " << t << std::endl;
          std::cout << "yaw: " << yaw.transpose() << std::endl;
          std::cout << "current quat_error: \n"
                    << quat_error.transpose() << std::endl;
          std::cout << "z_B: " << z_B.transpose() << std::endl;         
          std::cout << "current q: " << curr_quat.coeffs().transpose()
                    << std::endl;
          std::cout << "previous q: " << prev_quat.coeffs().transpose()
                    << std::endl;
        }

      }



      prev_quat = curr_quat;
      setpoint.state.q(curr_quat);

      extremum.thrusts.add(setpoint.input.thrusts);
      extremum.collectiveThrust.add(setpoint.input.collective_thrust);
    } else {

      // yaw << 0.0, 0.0, 0.0;
      quad.toStateWithTrueYaw(t, pvajs, yaw, setpoint);
      extremum.collectiveThrust.add(setpoint.input.collective_thrust);
    }
    extremum.tilt.add(setpoint.state.getTiltedAngle());
    extremum.omg.add(setpoint.input.omega);
    extremum.rpy.add(rad2deg(quaternionToEulerAnglesRPY(setpoint.state.q())));

    // Add a setpoint
    setpoints.push_back(setpoint);

    dt = std::min(T - t, sampleTimeSec);
    t += dt;
  }

  extremum.length = length;

  // Add the last setpoint
  pvajs = polys.getPVAJS(t);
  if (tilt_convention) {
    quad.toStateWithTiltYaw(t, pvajs, yaw, setpoint);
  } else {
    quad.toStateWithTrueYaw(t, pvajs, yaw, setpoint);
  }
  setpoints.push_back(setpoint);
  return extremum;
}

bool MincoSnapTrajectory::saveAllWaypoints(const std::string &filename) {
  if (!polys.valid()) {
    return false;
  }

  Eigen::VectorXd durations = polys.getDurations();
  Eigen::Matrix3Xd points = polys.getPoints();
  std::vector<double> timestamps;

  timestamps.push_back(0.0);
  for (int i = 0; i < durations.size(); ++i) {
    timestamps.push_back(timestamps.back() + durations[i]);
  }

  // fs::create_directory("/home/fsc1/chao/ros_ws/togt_ws/src/drone_common/droros/droros/results/cpc");
  std::ofstream file;
  file.open(filename.c_str());
  file.precision(4);

  file << "waypoints: [";
  for (int i{0}; i < points.cols(); ++i) {
    if (i < points.cols() - 1) {
      file << "[" << points.col(i).x() << ", " << points.col(i).y() << ", "
           << points.col(i).z() << "],\n            ";
    } else {
      file << "[" << points.col(i).x() << ", " << points.col(i).y() << ", "
           << points.col(i).z() << "]";
    }
  }
  file << "]\n\n";

  file << "timestamps: [";
  for (int i{0}; i < timestamps.size(); ++i) {
    if (i < timestamps.size() - 1) {
      file << timestamps[i] << ",\n            ";
    } else {
      file << timestamps[i];
    }
  }
  file << "]\n\n";

  file.precision();
  file.close();

  return true;
}

bool MincoSnapTrajectory::saveSegments(
    const std::string &filename,
    const std::vector<std::pair<int, int>> &segments) {
  if (!polys.valid()) {
    return false;
  }
  Eigen::VectorXd durations = polys.getDurations();
  Eigen::Matrix3Xd points = polys.getPoints();

  const int nSegments = segments.size();

  Eigen::VectorXd raceDurations;
  Eigen::Matrix3Xd raceWaypoints;
  raceDurations.resize(nSegments);
  raceWaypoints.resize(3, nSegments - 1);

  std::cout << "piece number: " << durations.size() << std::endl;

  for (int i = 0; i < nSegments; ++i) {
    if (segments[i].first < 0 || segments[i].second < 0) {
      return false;
    }

    std::cout << "segments[" << i << "]: " << segments[i].first << ", "
              << segments[i].second << std::endl;
  }

  int idx{0};
  for (int i{0}; i < nSegments; ++i) {
    double dur = durations.segment(segments[i].first, segments[i].second).sum();
    raceDurations[i] = dur;
    // std::cout << "idx: " << idx << " dur: " << dur << std::endl;
    idx += segments[i].second;
    if (i < nSegments - 1) {
      Eigen::Vector3d pos = points.col(idx);
      raceWaypoints.col(i) = pos;
    }
  }

  std::vector<double> timestamps;

  timestamps.push_back(0.0);
  for (int i = 0; i < raceDurations.size(); ++i) {
    timestamps.push_back(timestamps.back() + raceDurations[i]);
  }

  std::ofstream file;
  // fs::create_directory("/home/fsc1/chao/ros_ws/togt_ws/src/drone_common/droros/droros/results/cpc");
  file.open(filename.c_str());
  file.precision(4);

  file << "waypoints: [";
  for (int i{0}; i < raceWaypoints.cols(); ++i) {
    if (i < raceWaypoints.cols() - 1) {
      file << "[" << raceWaypoints.col(i).x() << ", "
           << raceWaypoints.col(i).y() << ", " << raceWaypoints.col(i).z()
           << "],\n            ";
    } else {
      file << "[" << raceWaypoints.col(i).x() << ", "
           << raceWaypoints.col(i).y() << ", " << raceWaypoints.col(i).z()
           << "]";
    }
  }
  file << "]\n\n";

  file << "timestamps: [";
  for (int i{0}; i < timestamps.size(); ++i) {
    if (i < timestamps.size() - 1) {
      file << timestamps[i] << ",\n            ";
    } else {
      file << timestamps[i];
    }
  }
  file << "]\n\n";

  file << "durations: [";
  for (int i{0}; i < raceDurations.size(); ++i) {
    if (i < raceDurations.size() - 1) {
      file << raceDurations[i] << ",\n            ";
    } else {
      file << raceDurations[i];
    }
  }
  file << "]";

  file.precision();
  file.close();

  return true;
}

bool MincoSnapTrajectory::saveSegments(const std::string &filename,
                                       const int piecesPerSegment) {
  if (!polys.valid()) {
    return false;
  }
  Eigen::VectorXd durations = polys.getDurations();
  Eigen::Matrix3Xd points = polys.getPoints();

  const double nPieces = durations.size();
  const double nSegments = nPieces / piecesPerSegment;

  std::cout << "nPieces: " << nPieces << std::endl;
  std::cout << "nSegments: " << nSegments << std::endl;

  Eigen::VectorXd raceDurations;
  Eigen::Matrix3Xd raceWaypoints;

  if (nSegments <= 1) {
    raceDurations.resize(1);
  } else {
    raceDurations.resize(nSegments);
    raceWaypoints.resize(3, nSegments - 1);
  }

  int idx{0};
  for (int i{0}; i < nSegments; ++i) {
    double dur = durations.segment(idx, piecesPerSegment).sum();
    raceDurations[i] = dur;
    std::cout << "idx: " << idx << " dur: " << dur << std::endl;
    idx += piecesPerSegment;
    if (i < nSegments - 1) {
      Eigen::Vector3d pos = points.col(idx);
      raceWaypoints.col(i) = pos;
    }
  }

  std::vector<double> timestamps;

  timestamps.push_back(0.0);
  for (int i = 0; i < raceDurations.size(); ++i) {
    timestamps.push_back(timestamps.back() + raceDurations[i]);
  }

  std::ofstream file;
  // fs::create_directory("/home/fsc1/chao/ros_ws/togt_ws/src/drone_common/droros/droros/results/cpc");
  file.open(filename.c_str());
  file.precision(4);

  file << "waypoints: [";
  for (int i{0}; i < raceWaypoints.cols(); ++i) {
    if (i < raceWaypoints.cols() - 1) {
      file << "[" << raceWaypoints.col(i).x() << ", "
           << raceWaypoints.col(i).y() << ", " << raceWaypoints.col(i).z()
           << "],\n            ";
    } else {
      file << "[" << raceWaypoints.col(i).x() << ", "
           << raceWaypoints.col(i).y() << ", " << raceWaypoints.col(i).z()
           << "]";
    }
  }
  file << "]\n\n";

  file << "timestamps: [";
  for (int i{0}; i < timestamps.size(); ++i) {
    if (i < timestamps.size() - 1) {
      file << timestamps[i] << ",\n            ";
    } else {
      file << timestamps[i];
    }
  }
  file << "]\n\n";

  file << "durations: [";
  for (int i{0}; i < raceDurations.size(); ++i) {
    if (i < raceDurations.size() - 1) {
      file << raceDurations[i] << ",\n            ";
    } else {
      file << raceDurations[i];
    }
  }
  file << "]";

  file.precision();
  file.close();

  return true;
}

bool MincoSnapTrajectory::save(const std::string &filename) {
  if (!valid() || setpoints.empty()) {
    return false;
  }

  if (!setpoints.front().input.isSingleRotorThrusts()) {
    // setpoint.input.thrusts;
  }

  // std::string directory;
  // const size_t last_slash_idx = filename.rfind('/');
  // if (std::string::npos != last_slash_idx)
  // {
  //     directory = filename.substr(0, last_slash_idx);
  // }
  // fs::create_directory(directory.c_str());

  std::ofstream file;
  file.open(filename.c_str());
  file << "t,p_x,p_y,p_z,q_x,q_y,q_z,q_w,v_x,v_y,v_z,w_x,w_y,w_z,"
       << "a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,"
       << "u_1,u_2,u_3,u_4,"
       << "jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z,"
       << "thrust\n";
  Eigen::Vector3d accRot = Eigen::Vector3d::Zero();
  for (const auto &setpoint : setpoints) {
    const double &t = setpoint.state.t;
    const Eigen::Vector3d &pos = setpoint.state.p;
    const Eigen::Vector3d &vel = setpoint.state.v;
    const Eigen::Vector3d &acc = setpoint.state.a;
    const Eigen::Vector3d &jer = setpoint.state.j;
    const Eigen::Vector3d &sna = setpoint.state.s;
    const Eigen::Vector4d &quat = setpoint.state.qx;
    const Eigen::Vector3d &omg = setpoint.input.omega;
    const Eigen::Vector4d &thrusts = setpoint.input.thrusts;
    // std::cout << "quat(0): " << quat(0) << std::endl;
    file << std::setprecision(5) << t << "," << std::setprecision(5) << pos(0)
         << "," << pos(1) << "," << pos(2) << "," << quat(1) << "," << quat(2)
         << "," << quat(3) << "," << quat(0) << "," << vel(0) << "," << vel(1)
         << "," << vel(2) << "," << omg(0) << "," << omg(1) << "," << omg(2)
         << "," << acc(0) << "," << acc(1) << "," << acc(2) << "," << accRot(0)
         << "," << accRot(1) << "," << accRot(2) << "," << thrusts(0) << ","
         << thrusts(1) << "," << thrusts(2) << "," << thrusts(3) << ","
         << jer(0) << "," << jer(1) << "," << jer(2) << "," << sna(0) << ","
         << sna(1) << "," << sna(2) << "," << setpoint.input.collective_thrust
         << "\n";
  }

  file.close();
  return true;
}

std::ostream &operator<<(std::ostream &os, const MincoSnapTrajectory &traj) {
  os.precision(4);
  // os << std::scientific;
  os << "MincoSnapTrajectory:\n"
     << "quad_name =      [" << traj.quad_name << "]\n"
     << "start_pos =      [" << traj.start_pvaj.col(0).transpose() << "]\n"
     << "end_pos =        [" << traj.end_pvaj.col(0).transpose() << "]\n"
     << "start_yaw =      [" << traj.start_yaw << "]\n"
     << "end_yaw =        [" << traj.end_yaw << "]\n"
     << "rotation_type =  [" << static_cast<int>(traj.rotation_type) << "]\n"
     << "heading_type =   [" << static_cast<int>(traj.heading_type) << "]\n"
     << "P:\n [" << traj.waypoints.transpose() << "]\n"
     << "T:\n [" << traj.durations.transpose() << "]" << std::endl;
  os.precision();
  // os.unsetf(std::ios::scientific);
  return os;
}

} // namespace drolib