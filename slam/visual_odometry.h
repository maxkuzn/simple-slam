#pragma once

#include "map.h"
#include "tracking.h"
#include "mapping.h"
#include "viewer.h"
#include "dataset.h"

#include <memory>
#include <string>

namespace slam {


class VisualOdometry {
 public:
  VisualOdometry(const std::string& config_path);

  void Init();

  void Run();

  bool Step();

  TrackingStatus GetTrackingStatus() const {
    return tracking_->GetStatus();
  }

 private:
  bool is_inited_ = false;
  const std::string config_path_;

  std::shared_ptr<Tracking> tracking_;
  std::shared_ptr<Mapping> mapping_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<Viewer> viewer_;
  std::shared_ptr<Dataset> dataset_;
};


}  // namespace slam

