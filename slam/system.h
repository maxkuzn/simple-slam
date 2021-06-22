#pragma once

#include "slam/component/tracker.h"
#include <opencv2/core/core.hpp>
#include "slam/core/frame.h"
#include <memory>
#include "slam/settings.h"
#include "slam/util/logger.h"

namespace slam {


class System {
 public:
  System(const SLAMSettings& settings) {
    SetLogLevel(settings.GetLogLevel());
    InfoLogS() << "Creating SLAM System";
    tracker_ = std::make_shared<Tracker>();
  }

  void Track(const std::shared_ptr<Frame>& frame) {
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
};


}  // namespace slam

