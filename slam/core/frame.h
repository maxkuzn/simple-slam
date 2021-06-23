#pragma once

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdexcept>
#include <slam/util/logger.h>
#include <vector>

namespace slam {

class ORBExtractor;

class Frame : public std::enable_shared_from_this<Frame> {
 public:
  Frame(const cv::Mat& left_img, const cv::Mat& right_img, double timestamp)
    : id_(next_frame_id_++)
    , img_(left_img)
    , right_img_(right_img)
    , timestamp_(timestamp)
  {
  }

  static std::shared_ptr<Frame> FromStereo(const cv::Mat& left_img,
                                           const cv::Mat& right_img,
                                           double timestamp) {
    return std::make_shared<Frame>(left_img, right_img, timestamp);
  }

  static std::shared_ptr<Frame> FromRGBD(const cv::Mat& img,
                                         const cv::Mat& depth,
                                         double timestamp) {
    (void) img;
    (void) depth;
    (void) timestamp;
    LogFatal() << "Not Implemented";
    return nullptr;
  }

  void ExtractFeatures(ORBExtractor& orb_extractor);

  double GetTimestamp() const {
    return timestamp_;
  }

 private:
  static size_t next_frame_id_;
  const size_t id_;
  cv::Mat img_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

  cv::Mat right_img_;
  cv::Mat depth_;
  const double timestamp_;
};

}  // namespace slam

