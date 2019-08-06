#include "mov_decoder.h"

int main(int argc, char **argv) {
  const char *filename = "/home/mitchell/SERDP_WS/serdp_osb_nov_2018/raw_data/"
                         "11-14-2018/3d2r2/vid_20181114_154433.mov";

  video_decode_example("/tmp/test%d.pgm", filename);
  return 0;
}
