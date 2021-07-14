#pragma once

#include "slam/core/frame.h"
#include "slam/core/map_point.h"
#include "slam/feature/orb_matcher.h"
#include "slam/util/logger.h"
#include "slam/algorithm/pose_optimizer.h"

#include <memory>
#include <opencv2/core/hal/interface.h>
namespace slam {

class Tracker {
 public:
  enum class EStatus : uint8_t {
    kNotInitialized,
    kOk,
    kLost,
  };

 public:
  Tracker(const std::shared_ptr<ORBMatcher>& orb_matcher)
      : orb_matcher_(orb_matcher)
  {
  }

  void Track(const std::shared_ptr<Frame>& frame) {
    if (status_ == EStatus::kNotInitialized) {
      Initialization(frame);
    } else {
      // Match
      LogInfo() << "Match frames";
      orb_matcher_->Match(prev_frame_, frame);
      size_t size = frame->GetKeyPointsSize();
      for (size_t i = 0; i != size; ++i) {
        auto mp = frame->GetMapPoint(i);
        if (mp) {
          LogInfo() << "Found match\n"
                    << "Index: " << i << '\n'
                    << "Coord: " << mp->GetCoordinates() << '\n';
        }
      }
      frame->SetPose(prev_frame_->GetPose());
      OptimizePose(frame);
      // Create MapPoints
      for (size_t i = 0; i != size; ++i) {
        auto mp = frame->GetMapPoint(i);
        if (!mp) {
          cv::Mat w_c = frame->GetKeyPointWorldCoordinates(i);
          if (!w_c.empty()) {
            mp = std::make_shared<MapPoint>(w_c);
            frame->SetMapPoint(i, mp);
          }
        }
      }
      LogDebug() << "Pose of new frame is \n" << frame->GetPose() << '\n';
      prev_frame_ = frame;
    }
  }

 private:
  void Initialization(const std::shared_ptr<Frame>& frame) {
    frame->SetPose(cv::Mat::eye(4, 4, CV_32F));
    size_t size = frame->GetKeyPointsSize();
    for (size_t i = 0; i != size; ++i) {
      cv::Mat w_c = frame->GetKeyPointWorldCoordinates(i);
      if (!w_c.empty()) {
        auto map_point = std::make_shared<MapPoint>(w_c);
        frame->SetMapPoint(i, map_point);
      }
    }
    prev_frame_ = frame;
    status_ = EStatus::kOk;
  }

 private:
  EStatus status_ = EStatus::kNotInitialized;
  std::shared_ptr<ORBMatcher> orb_matcher_;
  std::shared_ptr<Frame> prev_frame_ = nullptr;
};

}  // namespace slam

