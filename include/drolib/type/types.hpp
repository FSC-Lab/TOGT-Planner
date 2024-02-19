#pragma once

#include <Eigen/Dense>
#include "drolib/base/shape_base.hpp"

namespace drolib {

using PVAJ = Eigen::Matrix<double, 3, 4>;
using PVAJS = Eigen::Matrix<double, 3, 5>;

static constexpr int PATH_DIM = 3;
static constexpr int STATE_DIM = 4;
static constexpr int POLY_DEG = 2 * STATE_DIM - 1;
static constexpr int NUM_COEFF = POLY_DEG + 1;

}