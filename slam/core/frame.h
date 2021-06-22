#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

namespace slam {

class Frame : public std::enable_shared_from_this<Frame> {
 public:

 private:
  cv::Mat img;
  cv::Mat right_img;
  cv::Mat depth;
};

}  // namespace slam

