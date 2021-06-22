#pragma once

#include "slam/core/frame.h"
#include <memory>
namespace slam {

class Tracker {
 public:
  Tracker() {
  }

  void Track(const std::shared_ptr<Frame>& frame) {
  }


};

}  // namespace slam

