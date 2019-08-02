#include "SonarData.h"

class GPMF_player {

public:
  std::shared_ptr<SonarData> playbackGPMF(const std::string &filename,
                                          std::ofstream &output, int stopAfter);
};
