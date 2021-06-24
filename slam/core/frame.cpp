#include "frame.h"
#include "slam/feature/orb_extractor.h"
#include "slam/util/logger.h"
#include <memory>


namespace slam {

size_t Frame::next_frame_id_ = 0;


void Frame::ExtractFeatures(ORBExtractor& orb_extractor) {
  LogInfo() << "Extracting features for the frame " << id_;
  orb_extractor.Extract(img_, key_points_, descriptors_);
  map_points_.resize(key_points_.size(), nullptr);
}



}  // namespace slam

