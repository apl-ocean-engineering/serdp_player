#include "GPMF_player.h"

std::shared_ptr<SonarData>
GPMF_player::playbackGPMF(const std::string &filename, std::ofstream &output,
                          int stopAfter) {
  std::shared_ptr<liboculus::SonarPlayerBase> player(
      liboculus::SonarPlayerBase::OpenFile(filename));

  if (!player) {
    LOG(WARNING) << "Unable to open sonar file";
    return nullptr;
  }

  if (!player->open(filename)) {
    LOG(INFO) << "Failed to open " << filename;
    return nullptr;
  }

  int count = 0;
  std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
  std::shared_ptr<SonarData> sonarData;
  while (ping && !player->eof()) {

    ping->valid();

    if (output.is_open()) {
      auto const buffer(ping->buffer());
      output.write((const char *)buffer->ptr(), buffer->size());
    }

    count++;
    if ((stopAfter > 0) && (count >= stopAfter))
      break;

    ping = player->nextPing();
  }

  LOG(INFO) << count << " sonar packets decoded";

  return nullptr;
}
