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

class KITTIFrameStreamer : public IFrameStreamer {
 public:
  KITTIFrameStreamer(const std::string& path_to_sequence)
    : path_to_sequence_(path_to_sequence)
    , timestamps_file_(path_to_sequence + "/times.txt")
    , curr_idx_(0)
  {
    if (!timestamps_file_.is_open()) {
      LogFatal() << "Cannot open times.txt file";
      throw std::runtime_error("File not exits");
    }
  }

  std::shared_ptr<Frame> NextFrame() {
    if (timestamps_file_.eof()) {
      LogDebug() << "End of images sequence";
      return nullptr;
    }
    double timestamp;
    if (!(timestamps_file_ >> timestamp)) {
      if (timestamps_file_.eof()) {
        LogDebug() << "End of images sequence";
      } else {
        LogError() << "Cannot load frame " << curr_idx_;
      }
      return nullptr;
    }
    LogInfo() << "Loading frame " << curr_idx_;
    std::stringstream idx_str;
    idx_str << std::setfill('0') << std::setw(6) << curr_idx_;
    ++curr_idx_;
    std::string left_img_path = path_to_sequence_ + "/image_0/" + idx_str.str() + ".png";
    std::string right_img_path = path_to_sequence_ + "/image_1/" + idx_str.str() + ".png";

    cv::Mat left_img = cv::imread(left_img_path, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat right_img = cv::imread(right_img_path, CV_LOAD_IMAGE_UNCHANGED);
    if (left_img.empty()) {
      LogError() << "Unable to load image \"" << left_img_path << "\"";
      return nullptr;
    }
    if (right_img.empty()) {
      LogError() << "Unable to load image \"" << right_img_path << "\"";
      return nullptr;
    }

    return Frame::FromStereo(left_img, right_img, timestamp);
  }

 private:
  std::string path_to_sequence_;
  std::ifstream timestamps_file_;
  size_t curr_idx_;
};

}  // namespace slam

