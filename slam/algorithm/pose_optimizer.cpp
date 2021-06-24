#include "pose_optimizer.h"
#include "slam/core/camera.h"


#include <g2o/core/block_solver.h>
#include <Eigen/src/Core/Matrix.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/linear_solver_eigen.h>
// #include <g2o/types/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
// #include <g2o/solvers/linear_solver_dense.h>
// #include <g2o/types/types_seven_dof_expmap.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <sophus/se3.hpp>

namespace slam {



/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(std::istream&) override {
    return true;
  }

  virtual bool write(std::ostream&) const override {
    return true;
  }
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjection(const Eigen::Vector3d& pos,
                 const std::shared_ptr<Camera>& camera)
      : pos3d_(pos)
      , camera_(camera)
  {
  }

  virtual void computeError() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_pixel = camera_->K * (T * pos3d_);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  virtual void linearizeOplus() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * pos3d_;
    double fx = camera_->fx;
    double fy = camera_->fy;
    // double cx = camera_->cx;
    // double cy = camera_->cy;
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;
    _jacobianOplusXi
      << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
      0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
  }

  virtual bool read(std::istream&) override {
    return true;
  }

  virtual bool write(std::ostream&) const override {
    return true;
  }

private:
  Eigen::Vector3d pos3d_;
  std::shared_ptr<Camera> camera_;
};




void OptimizePose(const std::shared_ptr<slam::Frame>& frame) {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  VertexPose* vertex_pose = new VertexPose();
  vertex_pose->setId(0);
  vertex_pose->setEstimate(Sophus::SE3d());
  optimizer.addVertex(vertex_pose);
  
  size_t index = 1;
  size_t size = frame->GetKeyPointsSize();
  for (size_t i = 0; i != size; ++i) {
    auto mp = frame->GetMapPoint(i);
    if (mp) {
      Eigen::Vector2d pixel_coords;
      const auto& kp = frame->GetKeyPoint(i);
      pixel_coords << kp.pt.x, kp.pt.y;
      Eigen::Vector3d map_point_coords;
      map_point_coords << mp->GetCoordinates().at<float>(0),
                          mp->GetCoordinates().at<float>(1),
                          mp->GetCoordinates().at<float>(2);
      EdgeProjection* edge = new EdgeProjection(map_point_coords,
                                                frame->GetCamera());
      edge->setId(index++);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(pixel_coords);
      edge->setInformation(Eigen::Matrix2d::Identity());
      optimizer.addEdge(edge);
    }
  }

  // optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  auto pose = vertex_pose->estimate().matrix();
  cv::Mat cv_pose(4, 4, CV_32F);
  for (size_t i = 0; i != 4; ++i) {
    for (size_t j = 0; j != 4; ++j) {
      cv_pose.at<float>(i, j) = pose(i, j);
    }
  }
  frame->SetPose(cv_pose);
}

}  // namespace slam

