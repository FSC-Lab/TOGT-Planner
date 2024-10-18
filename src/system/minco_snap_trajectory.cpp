#include "drolib/system/minco_snap_trajectory.hpp"
#include "drolib/type/set_point.hpp"
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/writer.h"

namespace drolib {

MincoSnapTrajectory::MincoSnapTrajectory(const std::string quad_name, 
                    const QuadManifold& quad,
                    const TrajData& data,
                    const double start_yaw, const double end_yaw,
                    const std::string& name,
                    const RotationType rtype,
                    const HeadingType htype)
    : name(name),
      quad_name(quad_name),
      quad(quad),
      polys(data.traj),
      start_pvaj(polys.getPVAJ(0.0)),
      end_pvaj(polys.getPVAJ(data.T.sum())),
      waypoints(data.P),
      durations(data.T),
      start_yaw(start_yaw),
      end_yaw(end_yaw),
      rotation_type(rtype),
      heading_type(htype) {}

MincoSnapTrajectory::MincoSnapTrajectory(const std::string quad_name, 
                    const QuadManifold& quad,
                    const TrajData& data,
                    const double start_yaw,
                    const std::string& name)
    : name(name),
      quad_name(quad_name),
      quad(quad),
      polys(data.traj),
      start_pvaj(polys.getPVAJ(0.0)),
      end_pvaj(polys.getPVAJ(data.T.sum())),
      waypoints(data.P),
      durations(data.T),
      start_yaw(start_yaw),
      end_yaw(start_yaw),
      rotation_type(RotationType::TILT_HEADING),
      heading_type(HeadingType::FORWARD_HEADING) {}

TrajExtremum MincoSnapTrajectory::getSetpointVec(
    const double sampleTimeSec) {
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
  Eigen::Quaterniond lastTilt{1, 0, 0, 0};

  extremum.vel.add(0.0);
  extremum.vel.add(0.0);
  extremum.vel.add(polys.getMaxVel());
  extremum.acc.add(polys.getMaxAcc());


  double t{0.0};
  double dt{0.0};

  Eigen::Vector3d last_pos = polys.getPos(t);
  double length{0.0};

  for (int i = 0; i <= nSamples; ++i) {
    pvajs = polys.getPVAJS(t);

    Eigen::Vector3d pos = pvajs.col(0);
    length += (pos - last_pos).norm();
    last_pos = pos;

    //TODO: only support CONSTANT_HEADING and FORWARD_HEADING right now
    if (heading_type == HeadingType::FORWARD_HEADING) {
      yaw << getHeading(pvajs.col(2), pvajs.col(1), lastTilt, lastHeading), 0.0, 0.0;
    } else {
      yaw << start_yaw, 0.0, 0.0;
    }

    if (rotation_type == RotationType::TILT_HEADING) {
      quad.toStateWithTiltYaw(t, pvajs, yaw, setpoint);
      extremum.thrusts.add(setpoint.input.thrusts);
    } else {
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
  if (rotation_type == RotationType::TILT_HEADING) {
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
  for (size_t i=0; i < size_t(durations.size()); ++i) {
    timestamps.push_back(timestamps.back() + durations[i]);
  }

  std::ofstream file;
  file.open(filename.c_str());
  file.precision(4);

  file << "timestamps: [";
  for (size_t i{0}; i < size_t(timestamps.size()); ++i) {
    if (i < timestamps.size() - 1) {
      file << timestamps[i] << ",\n            ";
    } else {
      file << timestamps[i];
    }
  }
  file << "]\n\n";
  
  file << "waypoints: [";
  for (size_t i{0}; i < size_t(points.cols()); ++i) {
    if (i < size_t(points.cols() - 1)) {
      file << "[" << points.col(i).x() << ", "
                  << points.col(i).y() << ", " 
                  << points.col(i).z() << "],\n            ";
    } else {
      file << "[" << points.col(i).x() << ", "
                  << points.col(i).y() << ", " 
                  << points.col(i).z() << "]";
    }
  }
  file << "]\n\n";


  file.precision();
  file.close();

  return true;
}


bool MincoSnapTrajectory::saveSegments(const std::string &filename, const int piecesPerSegment) {
  if (!polys.valid()) {
    return false;
  }
  Eigen::VectorXd durations = polys.getDurations();
  Eigen::Matrix3Xd points = polys.getPoints();

  const double nPieces = durations.size();
  const int nSegments = static_cast<int>(nPieces / piecesPerSegment);

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

    idx += piecesPerSegment;
    if (i < nSegments - 1) {
      Eigen::Vector3d pos = points.col(idx);
      raceWaypoints.col(i) = pos;
    }
  }

  std::vector<double> timestamps;

  timestamps.push_back(0.0);
  for (size_t i=0; i < size_t(raceDurations.size()); ++i) {
    timestamps.push_back(timestamps.back() + raceDurations[i]);
  }

  std::ofstream file;
  // fs::create_directory("/home/fsc1/chao/ros_ws/togt_ws/src/drone_common/droros/droros/results/cpc");
  file.open(filename.c_str());
  file.precision(4);

  file << "waypoints: [";
  for (size_t i{0}; i < size_t(raceWaypoints.cols()); ++i) {
    if (i < size_t(raceWaypoints.cols() - 1)) {
      file << "[" << raceWaypoints.col(i).x() << ", "
                  << raceWaypoints.col(i).y() << ", " 
                  << raceWaypoints.col(i).z() << "],\n            ";
    } else {
      file << "[" << raceWaypoints.col(i).x() << ", "
                  << raceWaypoints.col(i).y() << ", " 
                  << raceWaypoints.col(i).z() << "]";
    }
  }
  file << "]\n\n";

  file << "timestamps: [";
  for (size_t i{0}; i < size_t(timestamps.size()); ++i) {
    if (i < size_t(timestamps.size() - 1)) {
      file << timestamps[i] << ",\n            ";
    } else {
      file << timestamps[i];
    }
  }
  file << "]\n\n";

  file << "durations: [";
  for (size_t i{0}; i < size_t(raceDurations.size()); ++i) {
    if (i < size_t(raceDurations.size() - 1)) {
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

namespace details {

void writeCSV(const std::filesystem::path& filename,
              const SetpointVector& setpoints) {
  std::ofstream file;
  file.open(filename.c_str());
  file << "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,"
       << "a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,"
       << "u_1,u_2,u_3,u_4,"
       << "jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z\n";
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
    file << std::setprecision(5) << t << "," << std::setprecision(5) << pos(0)
         << "," << pos(1) << "," << pos(2) << "," << quat(0) << "," << quat(1)
         << "," << quat(2) << "," << quat(3) << "," << vel(0) << "," << vel(1)
         << "," << vel(2) << "," << omg(0) << "," << omg(1) << "," << omg(2)
         << "," << acc(0) << "," << acc(1) << "," << acc(2) << "," << accRot(0)
         << "," << accRot(1) << "," << accRot(2) << "," << thrusts(0) << ","
         << thrusts(1) << "," << thrusts(2) << "," << thrusts(3) << ","
         << jer(0) << "," << jer(1) << "," << jer(2) << "," << sna(0) << ","
         << sna(1) << "," << sna(2) << "\n";
  }
  file.close();
}

rapidjson::Value ToJson(const Eigen::Ref<const Eigen::VectorXd>& v,
                             rapidjson::MemoryPoolAllocator<>& alloc) {
  rapidjson::Value res(rapidjson::kArrayType);
  for (int i = 0; i < v.size(); ++i) {
    res.PushBack(v.coeff(i), alloc);
  }
  return res;
}

void writeJSON(const std::filesystem::path& filepath,
               const SetpointVector& setpoints) {
  rapidjson::Document doc;

  doc.SetArray();
  auto& alloc = doc.GetAllocator();
  doc.Reserve(static_cast<rapidjson::SizeType>(setpoints.size()), alloc);
  for (const auto& setpoint : setpoints) {
    const double& t = setpoint.state.t;
    rapidjson::Value item(rapidjson::kObjectType);
    item.AddMember("t", t, alloc)
        .AddMember("p", ToJson(setpoint.state.p, alloc), alloc)
        .AddMember("v", ToJson(setpoint.state.v, alloc), alloc)
        .AddMember("a_lin", ToJson(setpoint.state.a, alloc), alloc)
        .AddMember("jerk", ToJson(setpoint.state.j, alloc), alloc)
        .AddMember("snap", ToJson(setpoint.state.s, alloc), alloc)
        .AddMember("q", ToJson(setpoint.state.qx, alloc), alloc)
        .AddMember("w", ToJson(setpoint.input.omega, alloc), alloc)
        .AddMember("u", ToJson(setpoint.input.thrusts, alloc), alloc);
    doc.PushBack(item, alloc);
  }

  struct FCloseWrapper {
    void operator()(FILE* fp) { std::fclose(fp); }
  };
  std::unique_ptr<FILE, FCloseWrapper> fp{std::fopen(filepath.c_str(), "w"),
                                          FCloseWrapper()};
  char buf[(1 << 16) - 1];
  rapidjson::FileWriteStream os{fp.get(), buf, sizeof buf};
  rapidjson::Writer ws(os);
  ws.SetMaxDecimalPlaces(5);
  doc.Accept(ws);
}
}  // namespace details

bool MincoSnapTrajectory::save(const std::string& filename) {
  if (!valid() || setpoints.empty()) {
    return false;
  }

  if (!setpoints.front().input.isSingleRotorThrusts()) {
    return false;
  }

  std::filesystem::path filepath(filename);

  if (filepath.extension() == ".csv") {
    details::writeCSV(filepath, setpoints);
  } else if (filepath.extension() == ".json") {
    details::writeJSON(filepath, setpoints);
  } else {
    return false;
  }

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