#include "drolib/race/race_params.hpp"

namespace drolib {

RaceParams::RaceParams(const std::string& directory, const std::string& filename) {
  if(!load(fs::path(directory), fs::path(filename))) {
    // logger_.error("Fail to load parameters");
    std::cout << "Fail to load parameters\n";
  }
}

bool RaceParams::load(const fs::path &directory, const fs::path &filename) {
  fs::path path = fs::path(directory) / fs::path(filename);
  if (!checkFile(directory, &path)) {
    // logger_.error("Main configuration file not found");
    std::cout << "Main configuration file not found\n";
    return false;
  }

  Yaml yaml{path};

  fs::path planningInit, planningRefine;
  fs::path lbfgsInit, lbfgsRefine, quadrotor;
  yaml["planningInit"].getIfDefined(planningInit);
  yaml["planningRefine"].getIfDefined(planningRefine);
  yaml["lbfgsInit"].getIfDefined(lbfgsInit);
  yaml["lbfgsRefine"].getIfDefined(lbfgsRefine);
  yaml["quadrotor"].getIfDefined(quadrotor);

  bool exist{true};
  exist &= checkFile(directory, &planningInit);
  exist &= checkFile(directory, &planningRefine);
  exist &= checkFile(directory, &lbfgsInit);
  exist &= checkFile(directory, &lbfgsRefine);
  exist &= checkFile(directory, &quadrotor);
  if (!exist) {
    // logger_.error("Config files not found!");
    std::cout << "Config files not found!\n";
    return false;
  }

  bool loaded{true};
  loaded &= lpinit.load(lbfgsInit) && lpinit.valid();
  loaded &= lprefine.load(lbfgsRefine) && lprefine.valid();
  loaded &= tpinit.load(planningInit) && tpinit.valid();
  loaded &= tprefine.load(planningRefine) && tprefine.valid();
  loaded &= qp.load(quadrotor) && qp.valid();
  if (!loaded) {
    // logger_.error("Error occurs when loading configuration files!");
    std::cout << "Error occurs when loading configuration files!\n";
    return false;
  }

  return true;
}


std::ostream& operator<<(std::ostream& os, const RaceParams& params) {
  os.precision(4);
  os << "RaceParams:\n";
  os << params.tprefine << "\n";
  os << params.lprefine << "\n";
  os << params.qp << std::endl;

  os.precision();
  return os;
}
}