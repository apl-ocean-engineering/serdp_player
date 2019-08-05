#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <vector>;

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <CLI/CLI.hpp>
#include <libg3logger/g3logger.h>

#include "liboculus/DataRx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/SonarPlayer.h"
#include "liboculus/StatusRx.h"

class SonarData {
  std::vector<float> frequency;
  std::vector<float> bearings;
  std::vector<float> ranges;
  std::vector<float> intensities;

public:
  SonarData(int nBearings, int nRanges);

  void set_frequency(float frequency);
  void bearing_push_back(std::shared_ptr<liboculus::SimplePingResult> ping);
  void ranges_push_back(std::shared_ptr<liboculus::SimplePingResult> ping);
};
