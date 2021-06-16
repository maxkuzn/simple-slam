#pragma once

#include <mutex>
#include <vector>
#include <memory>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <opencv2/core/core.hpp>


namespace slam {


class Feature;


class Frame {
 public:
  Frame() {
  }

  Frame(uint32_t id, double timestamp, const Sophus::SE3d& pose,
        const cv::Mat& left_img, const cv::Mat& right_img)
      : id_(id)
      , timestamp_(timestamp)
      , pose_(pose)
      , left_img_(left_img)
      , right_img_(right_img)
  {
  }

  void SetPose(const Sophus::SE3d& pose) {
    std::unique_lock<std::mutex> lock(mutex_);
    pose_ = pose;
  }

  Sophus::SE3d GetPose() {
    std::unique_lock<std::mutex> lock(mutex_);
    return pose_;
  }

  void SetAsKeyFrame();

  static std::shared_ptr<Frame> CreateFrame();

  void SetImages(cv::Mat&& left_img, cv::Mat&& right_img) {
    left_img_ = std::move(left_img);
    right_img_ = std::move(right_img);
  }

  const cv::Mat& LeftImage() const {
    return left_img_;
  }

  const auto& LeftFeatures() const {
    return left_features_;
  }

 private:
  std::mutex mutex_;

  uint32_t id_ = 0;
  bool is_keyframe_ = false;
  uint32_t keyframe_id_ = 0;
  double timestamp_;

  Sophus::SE3d pose_;

  cv::Mat left_img_;
  cv::Mat right_img_;

  std::vector<std::shared_ptr<Feature>> left_features_;
  std::vector<std::shared_ptr<Feature>> right_features_;
};


}  // namespace slam

