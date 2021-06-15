#pragma once


#include "frame.h"
#include "map.h"
#include "camera.h"

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace slam {


class Mapping {
 public:
  Mapping();

  void SetCameras(const std::shared_ptr<Camera>& camera_left,
                  const std::shared_ptr<Camera>& camera_right) {
    camera_left_ = camera_left;
    camera_right_ = camera_right;
  }

  void SetMap(const std::shared_ptr<Map>& map) {
    map_ = map;
  }

  void UpdateMap();

  void Stop();

 private:
  void RunningLoop();
  void Optimize(std::vector<Frame>& keyframes, std::vector<Feature>& landmarks);

 private:
  std::mutex mutex_;
  std::thread executing_thread_;

  std::condition_variable map_update_cv_;
  std::atomic<bool> stop_flag_;

  std::shared_ptr<Camera> camera_left_ = nullptr;
  std::shared_ptr<Camera> camera_right_ = nullptr;

  std::shared_ptr<Map> map_;
};


}  // namespace slam

