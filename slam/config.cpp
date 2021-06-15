#include "config.h"
#include <memory>
#include <opencv2/core/persistence.hpp>

namespace slam {
namespace util {


void Config::SetParameterFile(const std::string &filename) {
  if (!config_) {
    config_ = std::make_shared<Config>();
  }
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if (!config_->file_.isOpened()) {
    config_->file_.release();
    throw std::runtime_error("Couldn't open config file");
  }
}

Config::~Config() {
  if (file_.isOpened()) {
    file_.release();
  }
}

std::shared_ptr<Config> Config::config_ = nullptr;


}  // namespace util
}  // namespace slam

