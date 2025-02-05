#pragma once

#include <Eigen/Dense>
#include "drolib/base/shape_base.hpp"

namespace drolib {

using PVAJ3D = Eigen::Matrix<double, 3, 4>;
using PVAJS3D = Eigen::Matrix<double, 3, 5>;

using PVAJ1D = Eigen::Matrix<double, 1, 4>;
using PVAJS1D = Eigen::Matrix<double, 1, 5>;

static constexpr int PATH_DIM = 3;
static constexpr int YAW_DIM = 1;
static constexpr int STATE_DIM = 4;
static constexpr int POLY_DEG = 2 * STATE_DIM - 1;
static constexpr int NUM_COEFF = POLY_DEG + 1;

}