#pragma once

#include "slam/core/frame.h"
#include <memory>

namespace slam {


class IFrameStreamer {
 public:
  class Iterator {
   public:
    Iterator(IFrameStreamer& frame_streamer)
      : frame_streamer_(frame_streamer)
      , curr_frame_(nullptr)
    {
    }

    Iterator(IFrameStreamer& frame_streamer,
             const std::shared_ptr<Frame>& curr_frame)
      : frame_streamer_(frame_streamer)
      , curr_frame_(curr_frame)
    {
    }

    std::shared_ptr<Frame> operator*() {
      return curr_frame_;
    }

    Iterator& operator++() {
      curr_frame_ = frame_streamer_.NextFrame();
      return *this;
    }

    bool operator==(const Iterator& other) const {
      return &frame_streamer_ == &other.frame_streamer_ &&
             curr_frame_.get() == other.curr_frame_.get();
    }

   private:
    IFrameStreamer& frame_streamer_;
    std::shared_ptr<Frame> curr_frame_;
  };

 public:
  IFrameStreamer(float cx, float cy, float fx, float fy)
      : camera_(std::make_shared<const Camera>(cx, cy, fx, fy))
  {
  }

  virtual ~IFrameStreamer() = default;

  virtual std::shared_ptr<Frame> NextFrame() = 0;

  Iterator begin() {
    return Iterator(*this, NextFrame());
  }

  Iterator end() {
    return Iterator(*this);
  }

 protected:
  const std::shared_ptr<const Camera> camera_;
};


}  // namespace slam

