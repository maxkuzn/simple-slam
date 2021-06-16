#include "viewer.h"

#include "feature.h"
#include "slam/frame.h"

#include <array>
#include <chrono>
#include <functional>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sophus/se3.hpp>
#include <sophus/types.hpp>
#include <stdexcept>
#include <thread>

#include <pangolin/display/display.h>
#include <pangolin/display/opengl_render_state.h>

namespace slam {

Viewer::Viewer() {
  viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
  stop_flag_.store(true);
  viewer_thread_.join();
}

void Viewer::AddCurrentFrame(const std::shared_ptr<Frame>& curr_frame) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!map_) {
    throw std::runtime_error("Viewer: Map isn't initialized");
  }
  active_keyframes_ = map_->GetActiveKeyFrames();
  active_landmarks_ = map_->GetActiveMapPoints();
  map_updated_ = true;
}

void Viewer::ThreadLoop() {
  pangolin::CreateWindowAndBind("SLAM", 2014, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState vis_camera(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View& vis_display = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0 / 768.0)
                                .SetHandler(new pangolin::Handler3D(vis_camera));

  static constexpr std::array<float, 3> kGreen = {0, 1, 0};

  while (!pangolin::ShouldQuit() && !stop_flag_.load()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    vis_display.Activate(vis_camera);

    std::unique_lock<std::mutex> lock(mutex_);
    if (curr_frame_) {
      DrawFrame(curr_frame_, kGreen);
      FollowCurrentFrame(vis_camera);

      cv::Mat img = PlotFrameImage();
      cv::imshow("image", img);
      cv::waitKey(1);
    }

    if (map_) {
      DrawMapPoints();
    }

    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}


cv::Mat Viewer::PlotFrameImage() {
  cv::Mat img;
  cv::cvtColor(curr_frame_->LeftImage(), img, CV_GRAY2BGR);
  auto&& features = curr_frame_->LeftFeatures();
  for (size_t i = 0; i != features.size(); ++i) {
    auto map_point = features[i]->LockMapPoint();
    if (map_point) {
      auto f = features[i];
      cv::circle(img, f->Position().pt, 2, cv::Scalar(0, 255, 0), 2);
    }
  }
}


void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
  Sophus::SE3d T_w_c = curr_frame_->GetPose().inverse();
  pangolin::OpenGlMatrix m(T_w_c.matrix());
  vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(const std::shared_ptr<Frame>& frame,
                       const std::array<float, 3>& color) {
  Sophus::SE3d T_w_c = frame->GetPose().inverse();
  constexpr float sz = 1.0;
  constexpr int line_width = 2;
  constexpr float fx = 400;
  constexpr float fy = 400;
  constexpr float cx = 512;
  constexpr float cy = 384;
  constexpr float width = 1080;
  constexpr float height = 768;

  glPushMatrix();

  Sophus::Matrix4f m = T_w_c.matrix().template cast<float>();
  glMultMatrixf(static_cast<GLfloat*>(m.data()));

  glColor3f(color[0], color[1], color[2]);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glEnd();
  glPopMatrix();
}


void Viewer::DrawMapPoints() {
  constexpr std::array<float, 3> kRed = {1, 0, 0};
  for (auto& kf : active_keyframes_) {
    DrawFrame(kf.second, kRed);
  }

  glPointSize(2);
  glBegin(GL_POINTS);
  for (auto& landmark : active_landmarks_) {
    auto pos = landmark.second->GetPos();
    glColor3f(kRed[0], kRed[1], kRed[2]);
    glVertex3d(pos[0], pos[1], pos[2]);
  }
  glEnd();
}

}  // namespace slam

