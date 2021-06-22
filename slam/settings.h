#pragma once

#include "slam/util/logger.h"
#include <stdexcept>
#include <string>

namespace slam {

class SLAMSettings {
 public:
  SLAMSettings () {
  }

  void ReadFromFile(const std::string& file_path) {
    throw std::runtime_error("Not implemented");
  }

  void SetLogLevel(ELogLevel log_level) {
    log_level_ = log_level;
  }

  ELogLevel GetLogLevel() const {
    return log_level_;
  }

 private:
  ELogLevel log_level_;
};


}  // namespace slam

