#pragma once

#include <Eigen/Eigen>
#include "drolib/base/parameter_base.hpp"
#include "drolib/solver/lbfgs.hpp"

namespace drolib {

class LbfgsParams : public ParameterBase {
 public:
  LbfgsParams();

  using ParameterBase::load;
  bool load(const Yaml& yaml) override;
  bool valid() const override;
  friend std::ostream& operator<<(std::ostream& os, const LbfgsParams& params);

 public:
  lbfgs_parameter_t params;
};

}  // namespace drolib