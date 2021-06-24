#pragma once

#include "slam/core/frame.h"
#include "slam/frame_streamer/frame_streamer.h"
#include "slam/util/logger.h"
#include <fstream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <iomanip>


namespace slam {

class TUMFrameStreamer : public IFrameStreamer {
 public:
  TUMFrameStreamer(const std::string& path_to_sequence)
    : IFrameStreamer(1, 1, 1, 1)
    , path_to_sequence_(path_to_sequence)
    , associations_file_(path_to_sequence + "/associations.txt")
    , curr_idx_(0)
  {
    if (!associations_file_.is_open()) {
      LogFatal() << "Cannot open times.txt file";
      throw std::runtime_error("File not exits");
    }
  }

  std::shared_ptr<Frame> NextFrame() {
    if (associations_file_.eof()) {
      LogDebug() << "End of images sequence";
      return nullptr;
    }
    double timestamp;
    if (!(associations_file_>> timestamp)) {
      if (associations_file_.eof()) {
        LogDebug() << "End of images sequence";
      } else {
        LogError() << "Cannot load frame " << curr_idx_;
      }
      return nullptr;
    }
    std::string rgb_file;
    std::string depth_file;
    associations_file_ >> rgb_file;
    associations_file_ >> timestamp;
    associations_file_ >> depth_file;
    LogInfo() << "Loading frame " << curr_idx_;
    ++curr_idx_;
    std::string rgb_img_path = path_to_sequence_ + "/" + rgb_file;
    std::string depth_path = path_to_sequence_ + "/" + depth_file;

    cv::Mat rgb_img = cv::imread(rgb_img_path, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat depth = cv::imread(depth_path, CV_LOAD_IMAGE_UNCHANGED);
    if (rgb_img.empty()) {
      LogError() << "Unable to load image \"" << rgb_img_path << "\"";
      return nullptr;
    }
    if (depth.empty()) {
      LogError() << "Unable to load image \"" << depth_path << "\"";
      return nullptr;
    }

    return Frame::FromRGBD(rgb_img, depth, timestamp, camera_);
  }

 private:
  std::string path_to_sequence_;
  std::ifstream associations_file_;
  size_t curr_idx_;
};

}  // namespace slam

