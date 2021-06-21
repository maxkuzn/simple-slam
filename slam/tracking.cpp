#include "tracking.h"
#include "slam/config.h"
#include "viewer.h"
#include "mapping.h"
#include "feature.h"
#include "map_point.h"
#include <opencv2/features2d.hpp>
#include <sophus/se3.hpp>
#include <stdexcept>

namespace slam {

Tracking::Tracking()
  : num_features_(util::Config::Get<int>("num_features"))
  , num_features_init_(util::Config::Get<int>("num_features_init"))
  , detector_(cv::GFTTDetector::create(util::Config::Get<int>("num_features"),
                                   0.01, 20))
{
}

void Tracking::AddFrame(const std::shared_ptr<Frame>& frame) {
  current_frame_ = frame;
  switch (status_) {
  case TrackingStatus::kIniting:
    StereoInit();
    break;
  case TrackingStatus::kTrackingGood:
    Track();
    break;
  case TrackingStatus::kTrackingBad:
    Track();
    break;
  case TrackingStatus::kLost:
    Reset();
    break;
  }
  last_frame_ = current_frame_;
}

void Tracking::Track() {
  if (last_frame_) {
    current_frame_->SetPose(relative_motion_ * last_frame_->GetPose());
  }

  // int num_track_last = TrackLastFrame();
  TrackLastFrame();
  tracking_inliers_ = EstimateCurrentPose();

  if (tracking_inliers_ > kNumFeaturesTracking) {
    status_ = TrackingStatus::kTrackingGood;
  } else if (tracking_inliers_ > kNumFeaturesTrackingBad) {
    status_ = TrackingStatus::kTrackingBad;
  } else {
    status_ = TrackingStatus::kLost;
  }

  InsertKeyFrame();
  relative_motion_ = current_frame_->GetPose() * last_frame_->GetPose().inverse();

  if (viewer_) {
    viewer_->AddCurrentFrame(current_frame_);
  }
}

void Tracking::InsertKeyFrame() {
  if (tracking_inliers_ >= kNumFeaturesNeededForKeyFrame) {
    // don't insert keyframe
    return;
  }

  current_frame_->SetAsKeyFrame();
  map_->InsertKeyFrame(current_frame_);

  SetObservationsForKeyFrame();
  DetectFeatures();

  FindFeaturesInRight();
  TriangulateNewPoints();
  mapping_->UpdateMap();

  if (viewer_) {
    viewer_->UpdateMap();
  }
}


void Tracking::SetObservationsForKeyFrame() {
  for (auto& feature : current_frame_->LeftFeatures()) {
    auto map_point = feature->LockMapPoint();
    if (map_point) {
      map_point->AddObservation(feature);
    }
  }
}


uint32_t Tracking::TriangulateNewPoints() {
  std::array<Sophus::SE3d, 2> poses{camera_left_->pose(), camera_right_->pose()};
  Sophus::SE3d current_pose_Twc = current_frame_->GetPose().inverse();
  uint32_t count_triangulated_points = 0;
  auto&& left_features = current_frame_->LeftFeatures();
  throw std::runtime_error("TriangulateNewPoints: Not implemented");
  for (size_t i = 0; i < left_features.size(); ++i) {
  }
}




void Tracking::Reset() {
  throw std::runtime_error("Reset not implemented");
}

}  // namespace slam

