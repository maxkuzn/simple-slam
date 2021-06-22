#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

namespace slam {

class Frame : public std::enable_shared_from_this<Frame> {
 public:
  Frame() {
  }

  static std::shared_ptr<Frame> FromStereo(const cv::Mat& left_img,
                                           const cv::Mat& right_img,
                                           double timestamp) {
    (void) left_img;
    (void) right_img;
    (void) timestamp;
    return std::make_shared<Frame>();
  }

  static std::shared_ptr<Frame> FromRGBD(const cv::Mat& img,
                                         const cv::Mat& depth,
                                         double timestamp) {
    (void) img;
    (void) depth;
    (void) timestamp;
    return std::make_shared<Frame>();
  }

 private:
  cv::Mat img;
  cv::Mat right_img;
  cv::Mat depth;
};

}  // namespace slam

