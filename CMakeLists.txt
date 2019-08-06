#
# project: serdp_player
#
cmake_minimum_required(VERSION 3.5)

get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

fips_setup(PROJECT serdp_player)

list(APPEND OPENCV_COMPONENTS core highgui imgproc )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)



fips_begin_module( serdpplayer )

  fips_src( lib *.cpp )

  fips_deps( gpmf g3log g3logger oculus serdp_common)

  fips_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
   )


fips_end_module()


fips_add_subdirectory( tools/ )
# fips_add_subdirectory(src/...)
fips_finish()


