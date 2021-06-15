#pragma once

#include <mutex>
#include <list>
#include <memory>

#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>


namespace slam {


class Feature;


class MapPoint {
 public:
  MapPoint() {
  }

  MapPoint(uint32_t id, const Eigen::Vector3d& pos)
      : id_(id)
      , pos_(pos)
  {
  }


  Eigen::Vector3d GetPos() {
    std::unique_lock<std::mutex> lock(mutex_);
    return pos_;
  }

  void SetPos(const Eigen::Vector3d& pos) {
    std::unique_lock<std::mutex> lock(mutex_);
    pos_ = pos;
  }


  void AddObservation(const std::shared_ptr<Feature>& feature) {
    std::unique_lock<std::mutex> lock(mutex_);
    ++observed_times_;
    observations_.push_back(feature);
  }

  void RemoveObservation(const std::shared_ptr<Feature>& feature);

  std::list<std::weak_ptr<Feature>> GetObservations() {
    std::unique_lock<std::mutex> lock(mutex_);
    return observations_;
  }


  static std::shared_ptr<MapPoint> CreateMapPoint();

 private:
  std::mutex mutex_;

  uint32_t id_;
  bool is_outlier_ = false;
  uint32_t observed_times_ = 0;

  Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();

  std::list<std::weak_ptr<Feature>> observations_;
};


}  // namespace slam
