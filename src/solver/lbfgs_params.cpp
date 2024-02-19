#include "drolib/solver/lbfgs_params.hpp"

namespace drolib {

LbfgsParams::LbfgsParams() {}

bool LbfgsParams::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

#define GET_PARAM(name)                            \
  if (!yaml[#name].getIfDefined(name)) {           \
    std::cout << #name << " is not found" << "\n"; \
    return false;                                  \
  }

  double memorySize, past, minStep, maxLinesearch, maxIterations;
  double relCostTolerance, relGradTolerance;
  GET_PARAM(memorySize)
  GET_PARAM(past)
  GET_PARAM(minStep)
  GET_PARAM(maxLinesearch)
  GET_PARAM(maxIterations)
  GET_PARAM(relCostTolerance)
  GET_PARAM(relGradTolerance)

#undef GET_PARAM

  params.mem_size = memorySize;
  params.past = past;
  params.min_step = minStep;
  params.max_linesearch = maxLinesearch;
  params.max_iterations = maxIterations;
  params.g_epsilon = relGradTolerance;
  params.delta = relCostTolerance;

  return true;
}

bool LbfgsParams::valid() const {
  bool check = true;
  check &= std::isfinite(params.mem_size);
  check &= std::isfinite(params.past);
  check &= params.min_step >= 0.0;
  check &= params.max_linesearch >= 0;
  check &= params.max_iterations >= 0;
  check &= params.g_epsilon >= 0.0;
  check &= params.delta >= 0.0;

  return check;
}

std::ostream& operator<<(std::ostream& os, const LbfgsParams& params) {
  os.precision(6);
  os << std::scientific;
  os << "LbfgsParams:\n"
     << "mem_size =       [" << params.params.mem_size << "]\n"
     << "past =           [" << params.params.past << "]\n"
     << "min_step =       [" << params.params.min_step << "]\n"
     << "max_linesearch = [" << params.params.max_linesearch << "]\n"
     << "max_iterations = [" << params.params.max_iterations << "]\n"
     << "g_epsilon =      [" << params.params.g_epsilon << "]"
     << std::endl;
  os.precision();
  os.unsetf(std::ios::scientific);
  return os;
}

}