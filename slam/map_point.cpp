#include "map_point.h"
#include "feature.h"

#include <algorithm>
#include <memory>

namespace slam {

std::shared_ptr<MapPoint> MapPoint::CreateMapPoint() {
  static uint32_t next_id = 0;
  std::shared_ptr<MapPoint> map_point(new MapPoint());
  map_point->id_ = next_id++;
  return map_point;
}

void MapPoint::RemoveObservation(const std::shared_ptr<Feature>& feature) {
  std::unique_lock<std::mutex> lock(mutex_);

  auto it = std::find_if(observations_.begin(), observations_.end(),
      [&] (auto curr) {
        return curr.lock() == feature;
      }
  );

  if (it != observations_.end()) {
    observations_.erase(it);
    feature->ResetMapPoint();
    --observed_times_;
  }
}


}  // namespace slam

