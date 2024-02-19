#pragma once

#include <Eigen/Eigen>
#include "drolib/type/command.hpp"
#include "drolib/type/quad_state.hpp"

namespace drolib {

struct Setpoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Setpoint() = default;
  Setpoint(const QuadState& state, const Command& input)
      : state(state), input(input) {}

  QuadState state;
  Command input;
};

using SetpointVector = std::vector<Setpoint>;

}  // namespace drolib