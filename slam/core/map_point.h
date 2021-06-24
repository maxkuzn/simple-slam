#pragma once

#include <opencv2/core.hpp>

namespace slam {

class MapPoint {
 public:
  MapPoint(const cv::Mat& coord)
      : coord_(coord)
  {
  }

  const cv::Mat& GetCoordinates() const {
    return coord_;
  }

 private:
  cv::Mat coord_;
};

}  // namespace slam

