#pragma once

#include "slam/core/frame.h"
#include "slam/feature/orb_matcher.h"
#include "slam/util/logger.h"
#include <memory>
#include <opencv2/core/hal/interface.h>
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
      LogInfo() << "Match frames";
      auto matches = orb_matcher_->Match(prev_frame_, frame);
      frame->SetPose(prev_frame_->GetPose());
      prev_frame_ = frame;
    }
  }

 private:
  void Init(const std::shared_ptr<Frame>& frame) {
    frame->SetPose(cv::Mat::eye(4, 4, CV_32F));
    prev_frame_ = frame;
    status_ = EStatus::kOk;
  }

 private:
  EStatus status_ = EStatus::kNotInitialized;
  std::shared_ptr<ORBMatcher> orb_matcher_;
  std::shared_ptr<Frame> prev_frame_ = nullptr;
};

}  // namespace slam

