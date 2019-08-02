#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <CLI/CLI.hpp>
#include <libg3logger/g3logger.h>

#include "liboculus/DataRx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/SonarPlayer.h"
#include "liboculus/StatusRx.h"

class SonarPlayer {

public:
  int playbackSonarFile(const std::string &filename, std::ofstream &output,
                        int stopAfter);
};
