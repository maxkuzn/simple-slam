#pragma once

#include "slam/core/camera.h"
#include "slam/core/map_point.h"
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
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

  Frame(stereo_t,
        const cv::Mat& left_img, const cv::Mat& right_img, double timestamp,
        const std::shared_ptr<Camera>& camera)
    : id_(next_frame_id_++)
    , img_(left_img)
    , right_img_(right_img)
    , timestamp_(timestamp)
    , camera_(camera)
  {
    LogFatal() << "Not implemented depth computation";
    throw std::runtime_error("Not implemented");
  }

  Frame(rgbd_t,
        const cv::Mat& img, const cv::Mat& depth, double timestamp,
        const std::shared_ptr<Camera>& camera)
    : id_(next_frame_id_++)
    , img_(img)
    , depth_(depth)
    , timestamp_(timestamp)
    , camera_(camera)
  {
  }

 public:
  static std::shared_ptr<Frame> FromStereo(const cv::Mat& left_img,
                                           const cv::Mat& right_img,
                                           double timestamp,
                                           const std::shared_ptr<Camera>& camera) {
    return std::shared_ptr<Frame>(
                new Frame(stereo_t{},
                          left_img, right_img, timestamp,
                          camera));
  }

  static std::shared_ptr<Frame> FromRGBD(const cv::Mat& img,
                                         const cv::Mat& depth,
                                         double timestamp,
                                         const std::shared_ptr<Camera>& camera) {
    return std::shared_ptr<Frame>(
                new Frame(rgbd_t{},
                          img, depth, timestamp,
                          camera));
  }

  const cv::Mat& GetImage() const {
    return img_;
  }

  void ExtractFeatures(ORBExtractor& orb_extractor);

  double GetTimestamp() const {
    return timestamp_;
  }

  const cv::Mat& GetDescr() const {
    return descriptors_;
  }

  void SetPose(const cv::Mat pose) {
    Tcw_ = pose.clone();
    Rcw_ = Tcw_.rowRange(0,3).colRange(0,3);
    Rwc_ = Rcw_.t();
    tcw_ = Tcw_.rowRange(0,3).col(3);
    Ow_ = -Rcw_.t()*tcw_;
  }

  const cv::Mat& GetPose() const {
    return Tcw_;
  }

  size_t GetKeyPointsSize() const {
    return key_points_.size();
  }

  const cv::KeyPoint& GetKeyPoint(size_t index) const {
    return key_points_[index];
  }

  cv::Mat GetKeyPointWorldCoordinates(size_t index) const {
    const cv::KeyPoint kp = key_points_[index];
    const float v = kp.pt.y;
    const float u = kp.pt.x;
    const float d = depth_.at<float>(v, u);
    if (d <= 0) {
      return cv::Mat();
    }
    const float x = (u - camera_->cx) * d * camera_->fx_inv;
    const float y = (u - camera_->cy) * d * camera_->fy_inv;
    cv::Mat camera_coord = (cv::Mat_<float>(3, 1) << x, y, d);
    return Rwc_ * camera_coord + Ow_;
  }

  void SetMapPoint(size_t index, const std::shared_ptr<MapPoint>& map_point) {
    map_points_[index] = map_point;
  }

  const std::shared_ptr<MapPoint>& GetMapPoint(size_t index) const {
    return map_points_[index];
  }

  const std::shared_ptr<Camera>& GetCamera() const {
    return camera_;
  }

 private:
  static size_t next_frame_id_;
  const size_t id_;

  // Pose
  cv::Mat Tcw_;
  cv::Mat Rwc_;
  cv::Mat Rcw_;
  cv::Mat tcw_;
  cv::Mat Ow_;

  // Image and key points
  cv::Mat img_;
  std::vector<cv::KeyPoint> key_points_;
  std::vector<std::shared_ptr<MapPoint>> map_points_;
  cv::Mat descriptors_;

  cv::Mat right_img_;
  cv::Mat depth_;
  const double timestamp_;

  std::shared_ptr<Camera> camera_;
};

}  // namespace slam

