#include "slam/util/logger.h"
#include "slam/system.h"
#include "slam/settings.h"
#include "slam/frame_streamer/kitti_frame_streamer.h"

int main(int argc, char** argv) {
  std::string sequence_path = "/home/max/slam/ORB_SLAM2/data/kitti/dataset/sequences/06";
  slam::KITTIFrameStreamer frame_streamer(sequence_path);
  slam::SLAMSettings settings;
  settings.SetLogLevel(slam::ELogLevel::kInfo);
  slam::System slam(settings);

  size_t iter = 0;
  for (auto&& frame : frame_streamer) {
    ++iter;
    if (iter > 50) {
      break;
    }
    slam.Track(frame);
  }
}

