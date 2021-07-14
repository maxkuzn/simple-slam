#include "slam/util/logger.h"
#include "slam/system.h"
#include "slam/settings.h"
#include "slam/frame_streamer/tum_frame_streamer.h"

int main(int argc, char** argv) {
  std::string sequence_path = "/home/max/slam/datasets/tum/rgbd_dataset_freiburg1_xyz";
  slam::TUMFrameStreamer frame_streamer(sequence_path);
  slam::SLAMSettings settings;
  settings.SetLogLevel(slam::ELogLevel::kDebug);
  slam::System slam(settings);

  size_t iter = 0;
  for (auto&& frame : frame_streamer) {
    ++iter;
    if (iter > 500) {
      break;
    }
    slam.Track(frame);
  }
}

