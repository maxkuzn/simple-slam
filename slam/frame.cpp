#include "frame.h"
#include <memory>

namespace slam {


std::shared_ptr<Frame> Frame::CreateFrame() {
  static uint32_t next_id = 0;
  std::shared_ptr<Frame> frame(new Frame());
  frame->id_ = next_id++;
  return frame;
}

void Frame::SetAsKeyFrame() {
  static uint32_t next_keyframe_id = 0;
  is_keyframe_ = true;
  keyframe_id_ = next_keyframe_id++;
}


}  // namespace slam

