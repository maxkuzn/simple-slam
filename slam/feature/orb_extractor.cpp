#include "orb_extractor.h"

#include "slam/core/frame.h"

namespace slam {

void ORBExtractor::Extract(const cv::Mat& img,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::Mat& descriptors) {
  detector_->detectAndCompute(
      img,
      cv::noArray(),
      keypoints,
      descriptors);
}

}  // namespace slam

