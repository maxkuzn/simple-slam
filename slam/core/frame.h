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
 private:
  struct stereo_t {
  };

  struct rgbd_t {
  };

  Frame(stereo_t, const cv::Mat& left_img, const cv::Mat& right_img, double timestamp)
    : id_(next_frame_id_++)
    , img_(left_img)
    , right_img_(right_img)
    , timestamp_(timestamp)
  {
  }

  Frame(rgbd_t, const cv::Mat& img, const cv::Mat& depth, double timestamp)
    : id_(next_frame_id_++)
    , img_(img)
    , depth_(depth)
    , timestamp_(timestamp)
  {
  }

 public:
  static std::shared_ptr<Frame> FromStereo(const cv::Mat& left_img,
                                           const cv::Mat& right_img,
                                           double timestamp) {
    return std::make_shared<Frame>(stereo_t{}, left_img, right_img, timestamp);
  }

  static std::shared_ptr<Frame> FromRGBD(const cv::Mat& img,
                                         const cv::Mat& depth,
                                         double timestamp) {
    return std::make_shared<Frame>(rgbd_t{}, img, depth, timestamp);
  }

  void ExtractFeatures(ORBExtractor& orb_extractor);

  double GetTimestamp() const {
    return timestamp_;
  }

  const cv::Mat& GetDescr() const {
    return descriptors_;
  }

  void SetPose(const cv::Mat pose) {
    pose_ = pose.clone();
  }

  const cv::Mat& GetPose() const {
    return pose_;
  }

 private:
  static size_t next_frame_id_;
  const size_t id_;
  cv::Mat pose_;

  cv::Mat img_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

  cv::Mat right_img_;
  cv::Mat depth_;
  const double timestamp_;
};

}  // namespace slam

