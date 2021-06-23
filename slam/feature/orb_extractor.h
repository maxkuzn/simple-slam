#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
// #include <opencv2/xfeatures2d.hpp>

namespace slam {

class Frame;

class ORBExtractor {
 public:
  ORBExtractor() {
    detector_ = cv::ORB::create();
  }

  void Extract(const cv::Mat& img,
               std::vector<cv::KeyPoint>& keypoints,
               cv::Mat& descriptors);

 private:
  cv::Ptr<cv::ORB> detector_;
};


}  // namespace slam

