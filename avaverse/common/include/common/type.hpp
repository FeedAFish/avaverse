#pragma once

#include "Eigen/Core"
#include "common/config.hpp"

namespace avaverse {

using Skeleton = Eigen::Matrix<double, kNumPose, 3>;

using ColorChannel = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

};  // namespace avaverse
