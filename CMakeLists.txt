#
# project: serdp_player
#
cmake_minimum_required(VERSION 3.5)

get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

fips_setup(PROJECT serdp_player)

list(APPEND OPENCV_COMPONENTS core highgui imgproc )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

find_path( AVCODEC_INCLUDE_DIR libavcodec/avcodec.h )
find_library( AVCODEC_LIBRARY avcodec )

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

fips_begin_module( serdpplayer )

  fips_src( lib *.cpp )

  fips_deps( gpmf g3log g3logger oculus serdp_common activeobject)

  fips_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    #${AVCODEC_LIBRARY}
   )
  fips_libs(
     ${OpenCV_LIBS}
  )


fips_end_module()


fips_add_subdirectory( tools/ )
# fips_add_subdirectory(src/...)
fips_finish()


