#pragma once


#include "camera.h"
#include "frame.h"

#include <cstdint>
#include <string>
#include <memory>
#include <vector>


namespace slam {


class Dataset {
 public:
  Dataset(const std::string& dataset_path);

  void Init();

  std::shared_ptr<Frame> NextFrame();

  std::shared_ptr<Camera> GetCamera(uint32_t camera_id) const {
    return cameras_.at(camera_id);
  }

 private:
  const std::string dataset_path_;
  uint32_t curr_img_idx_ = 0;
  std::vector<std::shared_ptr<Camera>> cameras_;
};


}  // namespace slam

