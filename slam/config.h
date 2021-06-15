#pragma once


#include <opencv2/core/persistence.hpp>
#include <string>
#include <memory>


namespace slam {
namespace util {


class Config {
 public:
  ~Config();

  static void SetParameterFile(const std::string& filename);

  template <typename T>
  static T Get(const std::string& key) {
    return T(Config::config_->file_[key]);
  }

 private:
  Config() {
  }

  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
};


}  // namespace util
}  // namespace slam

