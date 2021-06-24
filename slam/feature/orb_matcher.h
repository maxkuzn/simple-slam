#pragma once

#include "slam/core/frame.h"
#include <memory>
#include <opencv2/core.hpp>
// #include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
namespace slam {

class ORBMatcher {
 public:
  ORBMatcher() {
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  }

  void Match(const std::shared_ptr<Frame>& train_frame,
             const std::shared_ptr<Frame>& query_frame) {
    std::vector<cv::DMatch> matches;
    matcher_->match(query_frame->GetDescr(), train_frame->GetDescr(), matches);
    std::vector<std::shared_ptr<MapPoint>> matched_points;
    matched_points.reserve(matches.size());
    float min_dist = matches[0].distance;
    float max_dist = matches[0].distance;
    for (size_t i = 0; i != matches.size(); ++i) {
      min_dist = std::min(min_dist, matches[i].distance);
      max_dist = std::max(max_dist, matches[i].distance);
    }
    for (size_t i = 0; i != matches.size(); ++i) {
      if (matches[i].distance <= std::max(2 * min_dist, 30.0f)) {
        auto mp = train_frame->GetMapPoint(matches[i].trainIdx);
        if (mp) {
          query_frame->SetMapPoint(matches[i].queryIdx, mp);
        }
      }
    }
    /*
    std::vector<cv::KeyPoint> matched1, matched2;
    for(size_t i = 0; i < nn_matches.size(); i++) {
        DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if(dist1 < nn_match_ratio * dist2) {
            matched1.push_back(kpts1[first.queryIdx]);
            matched2.push_back(kpts2[first.trainIdx]);
        }
    }
    vector<DMatch> good_matches;
    vector<KeyPoint> inliers1, inliers2;
    for(size_t i = 0; i < matched1.size(); i++) {
        Mat col = Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = matched1[i].pt.x;
        col.at<double>(1) = matched1[i].pt.y;
        col = homography * col;
        col /= col.at<double>(2);
        double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) +
                            pow(col.at<double>(1) - matched2[i].pt.y, 2));
        if(dist < inlier_threshold) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            good_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    */
  }

 private:
  cv::Ptr<cv::DescriptorMatcher> matcher_;
};

}  // namespace slam

