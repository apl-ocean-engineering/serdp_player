#
# project: serdp_player
#
cmake_minimum_required(VERSION 3.5)

get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

fips_setup(PROJECT serdp_player)

list(APPEND OPENCV_COMPONENTS core highgui imgproc videoio)
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

find_path( AVCODEC_INCLUDE_DIR libavcodec/avcodec.h )
find_library( AVCODEC_LIBRARY avcodec )

find_path( AVFORMAT_INCLUDE_DIR libavformat/avformat.h )
find_library( AVFORMAT_LIBRARY avformat )

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)


find_path(MEM_INCLUDE_DIR libavutil/mem.h)
find_library(MEM_LIBRARIES mem)


find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARIES swscale)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARIES avutil)
#find_package(FFmpeg REQUIRED)




## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

fips_begin_module( serdpplayer )

  fips_src( lib )

  fips_deps( gpmf g3log g3logger oculus serdp_common activeobject)

  fips_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    #${FFMPEG_INCLUDE_DIR}
    ${AVCODEC_INCLUDE_DIR}
    ${SWSCALE_INCLUDE_DIR}
    ${AVFORMAT_INCLUDE_DIR}
    #${AVUTIL_INCLUDE_DIR}
   )
  fips_libs(
     ${OpenCV_LIBS}
     #${FFMPEG_LIBRARIES}
     ${AVCODEC_LIBRARY}
     ${SWSCALE_LIBRARIES}
     ${AVFORMAT_LIBRARY}
     ${AVUTIL_LIBRARIES}
  )


fips_end_module()


fips_add_subdirectory( tools/ )
# fips_add_subdirectory(src/...)
fips_finish()


