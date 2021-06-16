#pragma once

#include <memory>

#include <opencv2/core/core.hpp>


namespace slam {


class Frame;
class MapPoint;


class Feature {
 public:
  Feature() {
  }

  Feature(const std::shared_ptr<Frame>& frame, const cv::KeyPoint& kp)
    : frame_(frame)
    , pos_(kp)
  {
  }

  bool IsOutlier() const {
    return is_outlier_;
  }

  void SetOuttiler() {
    is_outlier_ = true;
  }

  bool IsOnLeftImage() const {
    return is_on_left_image_;
  }

  void SetOnLeftImage() {
    is_on_left_image_ = true;
  }

  void SetOnRightImage() {
    is_on_left_image_ = false;
  }

  auto LockMapPoint() {
    return map_point_.lock();
  }

  void ResetMapPoint() {
    map_point_.reset();
  }

  const cv::KeyPoint& GetPosition() {
    return pos_;
  }

 private:
  std::weak_ptr<Frame> frame_;
  cv::KeyPoint pos_;
  std::weak_ptr<MapPoint> map_point_;

  bool is_outlier_ = false;
  bool is_on_left_image_;
};


}  // namespace slam
