#pragma once


#include "frame.h"
#include "map.h"
#include "map_point.h"

#include <thread>
#include <memory>

// #include <pangolin/display/opengl_render_state.h>
#include <opencv2/core/mat.hpp>
#include <pangolin/pangolin.h>
#include <unordered_map>


namespace slam {


class Viewer {
 public:
  Viewer();

  void SetMap(const std::shared_ptr<Map>& map) {
    map_ = map;
  }

  void Close();

  void AddCurrentFrame(const std::shared_ptr<Frame>& curr_frame);

  void UpdateMap();

 private:
  void ThreadLoop();

  void DrawFrame(const std::shared_ptr<Frame>& frame,
                 const std::array<float, 3>& color);

  void DrawMapPoints();

  void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

  cv::Mat PlotFrameImage();

 private:

  std::shared_ptr<Frame> curr_frame_ = nullptr;
  std::shared_ptr<Map> map_;

  std::mutex mutex_;
  std::thread viewer_thread_;
  std::atomic<bool> stop_flag_ = false;

  std::unordered_map<uint32_t, std::shared_ptr<Frame>> active_keyframes_;
  std::unordered_map<uint32_t, std::shared_ptr<MapPoint>> active_landmarks_;
  bool map_updated_ = false;
};


}  // namespace slam

