#pragma once

#include "drolib/base/parameter_base.hpp"
#include "drolib/planner/traj_params.hpp"
#include "drolib/solver/lbfgs_params.hpp"
#include "drolib/system/quadrotor_params.hpp"
#include "drolib/utils/logger.hpp"
#include "drolib/file/file_utils.hpp"
#include <Eigen/Eigen>


namespace drolib {

class RaceParams {
public:
  RaceParams() {};
  RaceParams(const std::string &directory, const std::string &filename);
  RaceParams(const QuadParams &qp, const TrajParams &tpinit,
             const TrajParams &tprefine, const LbfgsParams &lpinit,
             const LbfgsParams &lprefine)
      : qp(qp), tpinit(tpinit), tprefine(tprefine), lpinit(lpinit),
        lprefine(lprefine) {}
  bool load(const fs::path &directory, const fs::path &filename);
  QuadParams qp;
  TrajParams tpinit;
  TrajParams tprefine;
  LbfgsParams lpinit;
  LbfgsParams lprefine;

  friend std::ostream& operator<<(std::ostream& os, const RaceParams& track);
  
private:
  // Logger logger_{"RaceParams"};
};

} // namespace drolib