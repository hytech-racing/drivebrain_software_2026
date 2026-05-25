#pragma once

#include <Eigen/Dense>

#include "htx_ekf/state.hpp"

namespace htx_ekf
{

using StateVec = Eigen::Matrix<double, StateIndex::SIZE, 1>;

using StateMat = Eigen::Matrix<double, StateIndex::SIZE, StateIndex::SIZE>;

}  // namespace htx_ekf