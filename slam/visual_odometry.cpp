#include "visual_odometry.h"
#include "slam/config.h"
#include "slam/dataset.h"
#include "slam/map.h"
#include "slam/mapping.h"
#include "slam/tracking.h"
#include "slam/viewer.h"
#include <memory>

namespace slam {


VisualOdometry::VisualOdometry(const std::string& config_path)
    : config_path_(config_path)
{
}

void VisualOdometry::Init() {
  util::Config::SetParameterFile(config_path_);
  dataset_ = std::shared_ptr<Dataset>(
      new Dataset(util::Config::Get<std::string>("dataset_dir")));

  dataset_->Init();

  tracking_ = std::shared_ptr<Tracking>(new Tracking);
  mapping_ = std::shared_ptr<Mapping>(new Mapping);
  map_ = std::shared_ptr<Map>(new Map);
  viewer_ = std::shared_ptr<Viewer>(new Viewer);

  tracking_->SetMapping(mapping_);
  tracking_->SetMap(map_);
  tracking_->SetViewer(viewer_);
  tracking_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  mapping_->SetMap(map_);
  mapping_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  viewer_->SetMap(map_);
}

void VisualOdometry::Run() {
  while (Step()) {
  }

  mapping_->Stop();
  viewer_->Close();
}

bool VisualOdometry::Step() {
  auto new_frame = dataset_->NextFrame();
  if (!new_frame) {
    return false;
  }

  return tracking_->AddFrame(new_frame);
}


}  // namespace slam

