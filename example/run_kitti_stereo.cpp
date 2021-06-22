#include "slam/util/logger.h"
#include <memory>
#include <slam/system.h>
#include <slam/settings.h>

int main(int argc, char** argv) {
  slam::SLAMSettings settings;
  settings.SetLogLevel(slam::ELogLevel::kInfo);
  slam::System slam(settings);
  /*
  std::shared_ptr<slam::VisualOdometry> vo(
      new slam::VisualOdometry("./config/default.yaml"));
  assert(vo->Init());
  vo->Run();
  */
}

