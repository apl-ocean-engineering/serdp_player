#include <SonarPlayer.h>

int SonarPlayer::playbackSonarFile(const std::string &filename,
                                   std::ofstream &output, int stopAfter) {
  std::shared_ptr<liboculus::SonarPlayerBase> player(
      liboculus::SonarPlayerBase::OpenFile(filename));

  if (!player) {
    LOG(WARNING) << "Unable to open sonar file";
    return -1;
  }

  if (!player->open(filename)) {
    LOG(INFO) << "Failed to open " << filename;
    return -1;
  }

  int count = 0;
  std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
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

  return 0;
}
