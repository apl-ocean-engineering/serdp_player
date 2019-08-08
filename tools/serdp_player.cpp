#include "mov_decoder.h"
#include "serdp_common/OpenCVDisplay.h"
#include "serdp_common/PingDecoder.h"

#include <opencv2/highgui/highgui.hpp>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

  libg3logger::G3Logger logger(argv[0]);

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v,--verbose", verbosity,
               "Additional output (use -vv for even more!)");

  std::string ipAddr("auto");
  app.add_option("--ip", ipAddr,
                 "IP address of sonar or \"auto\" to automatically detect.");

  std::string outputFilename("");
  app.add_option("-o,--output", outputFilename,
                 "Saves raw sonar data to specified file.");

  std::string inputFilename("");
  app.add_option("-i,--input", inputFilename,
                 "Reads raw sonar data from specified file.   Plays file "
                 "contents rather than contacting \"real\" sonar on network.");

  int stopAfter = -1;
  app.add_option("-n,--frames", stopAfter, "Stop after (n) frames.");

  CLI11_PARSE(app, argc, argv);

  if (verbosity == 1) {
    logger.stderrHandle->call(&ColorStderrSink::setThreshold, INFO);
  } else if (verbosity > 1) {
    logger.stderrHandle->call(&ColorStderrSink::setThreshold, DEBUG);
  }
  std::ofstream output;

  if (!outputFilename.empty()) {
    LOG(DEBUG) << "Opening output file " << outputFilename;
    output.open(outputFilename, std::ios_base::binary | std::ios_base::out);

    if (!output.is_open()) {
      LOG(WARNING) << "Unable to open " << outputFilename << " for output.";
      exit(-1);
    }
  }

  if (!inputFilename.empty()) {
    std::shared_ptr<liboculus::SonarPlayerBase> player(
        liboculus::SonarPlayerBase::OpenFile(inputFilename));

    if (!player) {
      LOG(WARNING) << "Unable to open sonar file";
      return 0;
    }

    if (!player->open(inputFilename)) {
      LOG(INFO) << "Failed to open " << inputFilename;
      return 0;
    }

    int count = 0;
    std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
    std::shared_ptr<serdp_common::OpenCVDisplay> display;
    cv::VideoCapture cap("/home/mitchell/SERDP_WS/serdp_osb_nov_2018/raw_data/"
                         "11-15-2018/3d2r2/output.mp4");

    float video_time_init = cap.get(cv::CAP_PROP_POS_MSEC);
    float sonar_time_init;
    while (ping && !player->eof()) {
      cv::Mat frame;
      // Capture frame-by-frame
      cap >> frame;
      std::cout << "Video timestamp: "
                << cap.get(cv::CAP_PROP_POS_MSEC) - video_time_init
                << std::endl;
      if (ping->valid()) {
        serdp_common::PingDecoder pingDecoder;
        serdp_common::PingDecoder::SonarData sonarData =
            pingDecoder.pingPlayback(ping);
        if (count == 0) {
          sonar_time_init = sonarData.timestamp / 1000.0;
        } else {
          std::cout << "gopro init timestamp: " << sonar_time_init << std::endl;
          std::cout << "gopro timestamp: " << count << std::endl;
        }

        display->sonarDisplay(ping);
        cv::imshow("video_img", frame);

        cv::waitKey(1);
      }

      count++;
      if ((stopAfter > 0) && (count >= stopAfter))
        break;

      ping = player->nextPing();
    }
  }

  return 0;
}
