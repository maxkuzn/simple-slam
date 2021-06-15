#include "dataset.h"
#include "slam/camera.h"
#include "slam/frame.h"
#include <Eigen/src/Core/Matrix.h>
#include <fstream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace slam {


Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path)
{
}

void Dataset::Init() {
  std::ifstream fin(dataset_path_ + "/calib.txt");
  if (!fin) {
    throw std::runtime_error("File \"" + dataset_path_ + "/calib.txt\" not found");
  }

  for (size_t i = 0; i != 4; ++i) {
    char camera_name[3];
    for (size_t j = 0; j < 3; ++j) {
      fin >> camera_name[j];
    }
    double projection_data[12];
    for (size_t j = 0; j != 12; ++j) {
      fin >> projection_data[j];
    }
    Eigen::Matrix3d K;
    K << projection_data[0], projection_data[1], projection_data[2],
         projection_data[4], projection_data[5], projection_data[6],
         projection_data[8], projection_data[9], projection_data[10];
    Eigen::Vector3d t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t;
    K = K * 0.5;

    std::shared_ptr<Camera> curr_camera(
        new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(),
                   Sophus::SE3d(Sophus::SO3d(), t)));
    cameras_.push_back(curr_camera);
  }
  fin.close();
  curr_img_idx_ = 0;
}


std::shared_ptr<Frame> Dataset::NextFrame() {
  cv::Mat image_left;
  cv::Mat image_right;
  {
    std::stringstream ss;
    ss << dataset_path_ << "/image_" << 0 << "/"
       << std::setw(6) << std::setfill('0') << curr_img_idx_ << ".png";
    image_left = cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
  }
  {
    std::stringstream ss;
    ss << dataset_path_ << "/image_" << 1 << "/"
       << std::setw(6) << std::setfill('0') << curr_img_idx_ << ".png";
    cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
  }

  if (!image_left.data || !image_right.data) {
    throw std::runtime_error("Image not found");
  }

  cv::Mat image_left_resized;
  cv::Mat image_right_resized;
  cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
             cv::INTER_NEAREST);
  cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
             cv::INTER_NEAREST);

  auto frame = Frame::CreateFrame();
  frame->SetImages(std::move(image_left_resized), std::move(image_right_resized));
  ++curr_img_idx_;
  return frame;
}

}  // namespace slam

