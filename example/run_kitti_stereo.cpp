#include <memory>
#include <slam/visual_odometry.h>

int main(int argc, char** argv) {
  std::shared_ptr<slam::VisualOdometry> vo(
      new slam::VisualOdometry("./config/default.yaml"));
  assert(vo->Init());
  vo->Run();
}

