#include "drolib/race/race_track.hpp"

namespace drolib {

RaceTrack::RaceTrack(const fs::path& filename) {
  load(filename);
}

RaceTrack::RaceTrack(const QuadState& init, const QuadState& end) {
  segments.clear();
  gates.clear();
  initState = init;
  endState = end;
}

int RaceTrack::totalWaypoints() const {
  assert(corridors.size() == gates.size() + 1);
  int sum{0};
  for (size_t i{0}; i < corridors.size(); ++i) {
    sum += corridors[i]->size();
    if (i < size_t(gates.size())) {
      sum += gates[i]->size();
    }
  }
  return sum;
}

void RaceTrack::updateWaypoints(const TrajData &tdata) {
  assert(totalWaypoints() == tdata.totalPieces);
  int idx{0};
  for (size_t i{0}; i < size_t(corridors.size()); ++i) {
    for (auto &wp : corridors[i]->corridor) {
      wp.point = tdata.P.col(idx);
      idx++;
    }

    if (i < size_t(gates.size())) {
      for (auto &wp : gates[i]->corridor) {
        wp.point = tdata.P.col(idx);
        idx++;
      }
    }
  }
}

std::vector<Eigen::Vector3d> RaceTrack::getWaypoints() const {
  std::vector<Eigen::Vector3d> waypoints;
  for (size_t i{0}; i < size_t(corridors.size()); ++i) {
    for (const auto &wp : corridors[i]->corridor) {
      waypoints.push_back(wp.point);
    }
    if (i < size_t(gates.size())) {
      for (const auto &wp : gates[i]->corridor) {
        waypoints.push_back(wp.point);
      }
    }
  }
  return waypoints;
}

std::vector<Eigen::Vector3d> RaceTrack::getGatepoints() const {
  std::vector<Eigen::Vector3d> gatepoints;
  for (size_t i{0}; i < size_t(gates.size()); ++i) {
    if (gates[i]->corridor.size() == 1) {
      gatepoints.push_back(gates[i]->corridor.front().point);
    } else {
      gatepoints.push_back(gates[i]->corridor.front().point);
      gatepoints.push_back(gates[i]->corridor.back().point);
    }
  }
  return gatepoints;
}

bool RaceTrack::getData(const TrajData &prev, TrajData &cur) {
  if (prev.totalPieces == cur.totalPieces) {
    cur = prev;
    return true;
  }

  assignWaypoints(cur);

  PiecewisePolynomial<POLY_DEG> segment;
  int segmentIdx{0};
  int pointIdx{0}, timeIdx{0};
  for (size_t i{0}; i < corridors.size(); ++i) {
    segment = prev.traj.getPieces(segments[segmentIdx].first, segments[segmentIdx].second);
    segmentIdx++;
    const int midpoints = corridors[i]->size();
    double t{0};
    const double duration = segment.getTotalDuration() / (midpoints + 1);
    cur.T.segment(timeIdx, midpoints + 1).setConstant(duration);
    timeIdx += midpoints + 1;

    for (int j{0}; j < midpoints; ++j) {
      t += duration;
      cur.P.col(pointIdx++) = segment.getPos(t);
      // corridors[i]->corridor[j].point = segment.getPos(t);
    }

    if (i < size_t(gates.size())) {
      if (gates[i]->size() == 1) {
        t += duration;
        cur.P.col(pointIdx++) = segment.getPos(t);
        // gates[i]->corridor[0].point = segment.getPos(t);
      } else {
        segment = prev.traj.getPieces(segments[segmentIdx].first, segments[segmentIdx].second);
        segmentIdx++;

        const int totalpoints = gates[i]->size();
        double t{0};
        const double duration = segment.getTotalDuration() / (totalpoints - 1);
        cur.T.segment(timeIdx, totalpoints - 1).setConstant(duration);
        timeIdx += totalpoints - 1;

        for (int j{0}; j < totalpoints; ++j) {
          cur.P.col(pointIdx++) = segment.getPos(t);
          // gates[i]->corridor[j].point = segment.getPos(t);
          t += duration;
        }
      }

    }
  }

  cur.calcInitialVal();

  return true;
}

bool RaceTrack::getData(const double speedGuess, TrajData &tdata) {
  assignWaypoints(tdata);
  tdata.initData(initState.p, endState.p, speedGuess);
  return true;
}

void RaceTrack::assignWaypoints(TrajData &tdata) {
  int idx{0};
  tdata.clear();
  for (size_t i{0}; i < corridors.size(); ++i) {
    tdata.append(corridors[i]->corridor);

    segments.emplace_back(idx, corridors[i]->size() + 1);
    idx += corridors[i]->size() + 1;

    if (i < size_t(gates.size())) {
      tdata.append(gates[i]->corridor);

      if (gates[i]->size() >= 2) {
        segments.emplace_back(idx, gates[i]->size() - 1);
        idx += gates[i]->size() - 1;
      }
    }
  }
  tdata.allocateSpace();
}

// void RaceTrack::saveWaypoints(const std::string &filename) {
//   std::ofstream file;
//   file.open(filename.c_str());
//   file.precision(3);
//   file << "[";
//   for (size_t i{0}; i < gates.size(); ++i) {
//     if (i < gates.size() - 1) {
//       file << "[" << gates[i]->corridor.front().point.x() << ","
//                   << gates[i]->corridor.front().point.y() << "," 
//                   << gates[i]->corridor.front().point.z() << "],\n ";
//     } else {
//       file << "[" << gates[i]->corridor.front().point.x() << ","
//                   << gates[i]->corridor.front().point.y() << "," 
//                   << gates[i]->corridor.front().point.z() << "]";
//     }
//   }
//   file << "]";

//   file.precision();
//   file.close();
// }

void RaceTrack::save(const std::string &filename) {
  std::ofstream file;
  file.open(filename.c_str());
  
  initState.write("initState", file);
  endState.write("endState", file);
  file.precision(2);
  file << "orders: [";
  for (size_t i{0}; i < size_t(gates.size()); ++i) {
    if (i < size_t(gates.size()) - 1) {
      file << "'" << gates[i]->order << "', ";
    } else {
      file << "'" << gates[i]->order << "']\n\n";
    }
  }
  for (size_t i{0}; i < size_t(gates.size()); ++i) {
    // file << (*gates[i]);
    gates[i]->write(file);
  }
  file.precision();
  file.close();
}

bool RaceTrack::load(const fs::path& filename) {
  name = removeExtension(getBaseName(filename.generic_string()));
  return load(Yaml(filename));
}

bool RaceTrack::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  segments.clear();

  initState.load(yaml["initState"]);
  endState.load(yaml["endState"]);

  std::vector<std::string> orders;
  if (yaml["orders"].size() == 0) {
    std::cout << "No gate in the race course\n";
    gates.clear();
    return true;
  } else if (yaml["orders"].size() == 1) {
    std::string order = yaml["orders"].as<std::string>();
    orders.push_back(order);
  } else {
    yaml["orders"].getIfDefined(orders);
  }

  gates.clear();
  // gates.resize(orders.size());
  Yaml gateYaml;
  std::string type;
  for (const auto &order : orders) {
    gateYaml = yaml[order];

    if (!gateYaml["type"].getIfDefined(type)) {
      std::cout << order << " is not given!\n";
      return false;
    }

    // Add a new gate
    gates.push_back(createCorridor.at(type)(gateYaml, order));
  }

  return true;
}

void RaceTrack::initCorridors(const int midpoints) {
  std::vector<Eigen::Vector3d> startPoints;
  std::vector<Eigen::Vector3d> endPoints;
  startPoints.resize(gates.size() + 1);
  endPoints.resize(gates.size() + 1);

  bool ret = true;
  startPoints[0] = initState.p;
  for (size_t i{0}; i < gates.size(); ++i) {
    ret &= gates[i]->lastPoint(startPoints[i + 1]);
    ret &= gates[i]->firstPoint(endPoints[i]);
  }
  endPoints[gates.size()] = endState.p;

  corridors.clear();
  // corridors.resize(gates.size() + 1);
  for (size_t i{0}; i < gates.size() + 1; ++i) {
    const Eigen::Vector3d &p0 = startPoints[i];
    const Eigen::Vector3d &p1 = endPoints[i];
    // initCorridor(i, p0, p1, midpoints);
    corridors.push_back(std::make_shared<FreeCorridor>(p0, p1, midpoints, minCorDist));
  }
}

std::ostream& operator<<(std::ostream& os, const RaceTrack& track) {
  os.precision(4);
  os << "RaceTrack:\n";
  os << track.initState << "\n";
  os << track.endState << "\n";

  if (!track.corridors.empty()) {
    for (size_t i{0}; i < track.corridors.size(); ++i) {
      os << i << " corridor--------------------\n";
      for (const auto &wp : track.corridors[i]->corridor) {
        os << "   name: " << wp.shape->name << ", pos: " << wp.point.transpose() << "\n";
      }
      if (i < track.gates.size()) {
        for (const auto &wp : track.gates[i]->corridor) {
          os << "   name: " << wp.shape->name << ", pos: " << wp.point.transpose() << "\n";
        }
      }
    }
  } else if (!track.gates.empty()) {
    for (size_t i{0}; i < track.gates.size(); ++i) {
      for (const auto &wp : track.gates[i]->corridor) {
        os << "   name: " << wp.shape->name << ", pos: " << wp.point.transpose() << "\n";
      }
    }
  } else {
    os << "No gate nor corridor available.\n";
  }

  os.precision();
  return os;
}


} // namespace drolib

