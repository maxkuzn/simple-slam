#pragma once

#include <Eigen/Core>

namespace slam {

struct Camera {
  Camera(float cx_, float cy_, float fx_, float fy_)
      : cx(cx_)
      , cy(cy_)
      , fx(fx_)
      , fy(fy_)
      , fx_inv(1.0f / fx_)
      , fy_inv(1.0f / fy_)
  {
    K << fx,  0, cx,
          0, fy, cy,
          0,  0,  1;
    /*
    K << 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0;
    */
  }

  const float cx;
  const float cy;
  const float fx;
  const float fy;
  const float fx_inv;
  const float fy_inv;
  Eigen::Matrix3d K;
};

}  // namespace slam

