#pragma once

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

 private:
};


}  // namespace slam

