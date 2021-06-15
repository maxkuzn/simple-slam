#pragma once


#include <memory>
#include <sophus/se3.hpp>
#include <opencv2/features2d.hpp>

#include "map.h"
#include "camera.h"


namespace slam {


class Mapping;
class Viewer;


enum class TrackingStatus {
  kIniting,
  kTrackingGood,
  kTrackingBad,
  kLost,
};

class Tracking {
 public:
  Tracking();

  bool AddFrame(const std::shared_ptr<Frame>& frame);

  void SetMap(const std::shared_ptr<Map>& map);

  void SetMapping(const std::shared_ptr<Mapping>& mapping) {
    mapping_ = mapping;
  }

  void SetViewer(const std::shared_ptr<Viewer>& viewer) {
    viewer_ = viewer;
  }

  TrackingStatus GetStatus() const {
    return status_;
  }

  void SetCameras(const std::shared_ptr<Camera>& left,
                 const std::shared_ptr<Camera>& right) {
    camera_left_ = left;
    camera_right_ = right;
  }


 private:
  bool Track();
  bool Reset();

  // Returns num of tracked points
  uint32_t TrackLastFrame();

  // Returns num of inliers 
  uint32_t EstimateCurrentPose();

  bool InsertKeyFrame();

  bool StereoInit();

  uint32_t DetectFeatures();

  uint32_t FindFeaturesInRight();

  bool BuildInitMap();

  uint32_t TriangulateNewPoints();

  void SetObservationsForKeyFrame();

 private:
  TrackingStatus status_ = TrackingStatus::kIniting;

  std::shared_ptr<Frame> current_frame_ = nullptr;
  std::shared_ptr<Frame> last_frame_ = nullptr;

  std::shared_ptr<Camera> camera_left_ = nullptr;
  std::shared_ptr<Camera> camera_right_ = nullptr;

  std::shared_ptr<Map> map_ = nullptr;
  std::shared_ptr<Mapping> mapping_ = nullptr;
  std::shared_ptr<Viewer> viewer_ = nullptr;

  Sophus::SE3d relative_motion_;

  cv::Ptr<cv::GFTTDetector> detector_;

  uint32_t tracking_inliers_ = 0;

  static constexpr uint32_t kNumFeatures = 200;
  static constexpr uint32_t kNumFeaturesInit = 100;
  static constexpr uint32_t kNumFeaturesTracking = 50;
  static constexpr uint32_t kNumFeaturesTrackingBad = 20;
  static constexpr uint32_t kNumFeaturesNeededForKeyFrame = 80;
};


}  // namespace slam

