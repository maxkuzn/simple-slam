#pragma once


#include <sophus/se3.hpp>
#include <Eigen/Core>


namespace slam {


class Camera {
 public:
  Camera(double fx, double fy, double cx, double cy, double baseline,
         const Sophus::SE3d& pose)
      : fx_(fx)
      , fy_(fy)
      , cx_(cx)
      , cy_(cy)
      , baseline_(baseline)
      , pose_(pose)
      , pose_inv_(pose.inverse())
  {
  }

  const Sophus::SE3d pose() const {
    return pose_;
  }

  Eigen::Matrix3d K() const {
    Eigen::Matrix3d k;
    k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    return k;
  }

  Eigen::Vector3d world2camera(const Eigen::Vector3d& p_w,
                               const Sophus::SE3d& T_c_w);

  Eigen::Vector3d camera2world(const Eigen::Vector3d& p_c,
                               const Sophus::SE3d& T_c_w);

  Eigen::Vector3d camera2pixel(const Eigen::Vector3d& p_c);

  Eigen::Vector3d pixel2camera(const Eigen::Vector3d& p_p, double depth);

  Eigen::Vector3d pixel2world(const Eigen::Vector3d& p_p,
                              const Sophus::SE3d& T_c_w, double depth);

  Eigen::Vector3d world2pixel(const Eigen::Vector3d& p_2,
                              const Sophus::SE3d& T_c_w);

 private:
  const double fx_;
  const double fy_;
  const double cx_;
  const double cy_;
  const double baseline_;
  const Sophus::SE3d pose_;
  const Sophus::SE3d pose_inv_;
};


}  // namespace slam

