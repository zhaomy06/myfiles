#pragma once
#include <Eigen/Core>
#include <vector>

#include "ilqr_config.pb.h"

static constexpr int kMaxBackwardPassCount = 10;
static constexpr double kLambdaFactor = 2;
static constexpr double kLambdaMax = 100.0;
static constexpr double kLambdaMin = 1e-5;
static constexpr double kLambdaFix = 0.1;
static constexpr double kZMin = 0.0;
static std::vector<double> alpha_vec = {1.0000, 0.6180, 0.3819, 0.2360, 0.1458,
                                        0.0901, 0.0557, 0.0344, 0.01};
