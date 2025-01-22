#pragma once

#include "drolib/base/parameter_base.hpp"
#include "drolib/corridor/free_corridor.hpp"
#include "drolib/corridor/prisma_corridor.hpp"
#include "drolib/corridor/single_ball.hpp"
#include "drolib/file/file_utils.hpp"
#include "drolib/planner/traj_data.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"
#include "drolib/type/quad_state.hpp"
#include "drolib/utils/logger.hpp"
#include <unordered_map>

namespace drolib {
class RaceTrack : public ParameterBase {
public:

  explicit RaceTrack(const fs::path &filename);
  RaceTrack(const QuadState &init, const QuadState &end);
  RaceTrack() = default;
  ~RaceTrack() = default;

  bool isChanged() const {
    for (size_t i{0}; i < size_t(gates.size()); ++i) {
      if (gates[i]->isChanged()) {
        return true;
      }
    }
    return false;
  }

  void clearChange() {
    for (size_t i{0}; i < size_t(gates.size()); ++i) {
      gates[i]->clearChange();
    }
  }

  bool valid() const override {
    if (corridors.empty() || corridors.size() - 1 != gates.size()) {
      std::cout
          << "There should be at least one corridor, even if it is empty.\n";
      return false;
    }
    return true;
  }

  bool empty() const { return gates.empty(); }

  int totalWaypoints() const;

  void updateWaypoints(const TrajData &tdata);

  bool getData(const TrajData &prev, TrajData &cur);

  bool getData(const double speedGuess, TrajData &tdata);

  void assignWaypoints(TrajData &tdata);

  bool load(const Yaml &yaml) override;

  bool load(const fs::path &filename) override;

  void save(const std::string &filename);

  // void saveWaypoints(const std::string &filename);

  void initCorridors(const int midpoints = 0);

  inline std::string getName() const { return name; }

  inline std::vector<std::pair<int, int>> getSegments() const {
    return segments;
  }

  std::vector<Eigen::Vector3d> getWaypoints() const;

  std::vector<Eigen::Vector3d> getGatepoints() const;

  friend std::ostream &operator<<(std::ostream &os, const RaceTrack &track);

  std::unordered_map<std::string,
                     std::function<std::shared_ptr<CorridorBase>(
                         const Yaml &yaml, const std::string &order)>>
      createCorridor{
          {"FreeCorridor",
           [](const Yaml &yaml, const std::string &order) {
             return std::make_shared<FreeCorridor>(yaml, order);
           }},
          {"SingleBall",
           [](const Yaml &yaml, const std::string &order) {
             return std::make_shared<SingleBall>(yaml, order);
           }},
          {"TrianglePrisma",
           [](const Yaml &yaml, const std::string &order) {
             return std::make_shared<PrismaCorridor>("TrianglePrisma", yaml,
                                                     order);
           }},
          {"RectanglePrisma",
           [](const Yaml &yaml, const std::string &order) {
             return std::make_shared<PrismaCorridor>("RectanglePrisma", yaml,
                                                     order);
           }},
          {"PentagonPrisma",
           [](const Yaml &yaml, const std::string &order) {
             return std::make_shared<PrismaCorridor>("PentagonPrisma", yaml,
                                                     order);
           }},
          {"HexagonPrisma", [](const Yaml &yaml, const std::string &order) {
             return std::make_shared<PrismaCorridor>("HexagonPrisma", yaml,
                                                     order);
           }}};

public:
  std::string name;
  std::vector<std::shared_ptr<CorridorBase>> gates;
  std::vector<std::shared_ptr<CorridorBase>> corridors;
  std::vector<std::pair<int, int>> segments;
  QuadState initState;
  QuadState endState;
  double minCorDist{0.0};

private:
  Logger logger_{"RaceTrack"};
};

} // namespace drolib
