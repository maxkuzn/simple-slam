#pragma once

#include <opencv2/core/core.hpp>
#include <slam/core/frame.h>
#include <memory>
#include <slam/settings.h>

namespace slam {


class System {
 public:
  System(const SLAMSettings& settings) {
  }

  void Track(const std::shared_ptr<Frame>& frame) {
  }

  void TrackStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timestamp) {
  }

  void TrackRGBD(const cv::Mat& img, const cv::Mat& depth, double timestamp) {
  }

 private:
};


}  // namespace slam

