#pragma once

#include "slam/component/tracker.h"
#include <opencv2/core/core.hpp>
#include "slam/core/frame.h"
#include <memory>
#include "slam/feature/orb_matcher.h"
#include "slam/settings.h"
#include "slam/util/logger.h"
#include "slam/feature/orb_extractor.h"

namespace slam {


class System {
 public:
  System(const SLAMSettings& settings) {
    SetLogLevel(settings.GetLogLevel());
    LogInfo() << "Creating SLAM System";
    orb_extractor_ = std::make_shared<ORBExtractor>();
    orb_matcher_ = std::make_shared<ORBMatcher>();
    tracker_ = std::make_shared<Tracker>(orb_matcher_);
  }

  void Track(const std::shared_ptr<Frame>& frame) {
    frame->ExtractFeatures(*orb_extractor_);
    tracker_->Track(frame);
  }

  void TrackStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timestamp) {
    std::shared_ptr<Frame> frame = Frame::FromStereo(left_img, right_img, timestamp);
    Track(frame);
  }

  void TrackRGBD(const cv::Mat& img, const cv::Mat& depth, double timestamp) {
    std::shared_ptr<Frame> frame = Frame::FromRGBD(img, depth, timestamp);
    Track(frame);
  }

 private:
  std::shared_ptr<Tracker> tracker_;
  std::shared_ptr<ORBExtractor> orb_extractor_;
  std::shared_ptr<ORBMatcher> orb_matcher_;
};


}  // namespace slam

