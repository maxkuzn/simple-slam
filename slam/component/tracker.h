#pragma once

#include "slam/core/frame.h"
#include "slam/feature/orb_matcher.h"
#include <memory>
namespace slam {

class Tracker {
 public:
  enum class EStatus : uint8_t {
    kNotInitialized,
    kOk,
    kLost,
  };

 public:
  Tracker(const std::shared_ptr<ORBMatcher>& orb_matcher)
      : orb_matcher_(orb_matcher)
  {
  }

  void Track(const std::shared_ptr<Frame>& frame) {
    if (status_ == EStatus::kNotInitialized) {
      Init(frame);
    } else {
      // Match
    }
  }

 private:
  void Init(const std::shared_ptr<Frame>& frame) {
  }

 private:
  EStatus status_ = EStatus::kNotInitialized;
  std::shared_ptr<ORBMatcher> orb_matcher_;
};

}  // namespace slam

