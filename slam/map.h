#pragma once

#include "frame.h"
#include "map_point.h"


namespace slam {


class Map {
 public:
  Map() {
  }

  void InsertKeyFrame(const std::shared_ptr<Frame>& frame);

  void InsertMapPoint(const std::shared_ptr<MapPoint>& map_point);

  auto GetAllMapPoints() {
    std::unique_lock<std::mutex> lock(mutex_);
    return map_points_;
  }

  auto GetActiveMapPoints() {
    std::unique_lock<std::mutex> lock(mutex_);
    return active_map_points_;
  }

  auto GetAllKeyFrames() {
    std::unique_lock<std::mutex> lock(mutex_);
    return key_frames_;
  }

  auto GetActiveKeyFrames() {
    std::unique_lock<std::mutex> lock(mutex_);
    return active_key_frames_;
  }

  void Clear();

 private:
  static constexpr size_t kNumActiveKeyFrames = 7;

  void RemoveOldKeyFrame();

  std::mutex mutex_;

  std::unordered_map<uint32_t, std::shared_ptr<MapPoint>> map_points_;
  std::unordered_map<uint32_t, std::shared_ptr<MapPoint>> active_map_points_;
  std::unordered_map<uint32_t, std::shared_ptr<Frame>> key_frames_;
  std::unordered_map<uint32_t, std::shared_ptr<Frame>> active_key_frames_;
};


}  // namespace slam

