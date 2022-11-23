#pragma once

#include "Eigen/Core"
#include "avaverse/config.hpp"
#include "unsupported/Eigen/CXX11/Tensor"

namespace avaverse {

using Skeleton = Eigen::Matrix<double, kNumPose, kDim>;

using ColorChannel = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

using EigenImage = Eigen::Tensor<uint8_t, 3>;

};  // namespace avaverse
