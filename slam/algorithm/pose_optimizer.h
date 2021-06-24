#pragma once


#include "slam/core/frame.h"
#include <memory>

namespace slam {

void OptimizePose(const std::shared_ptr<slam::Frame>& frame);

}  // namespace slam

