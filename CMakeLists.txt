#
# project: serdp_player
#
cmake_minimum_required(VERSION 3.5)
project(serdp_player)


if( FIPS_CONFIG AND NOT FIPS_IMPORT )
  cmake_minimum_required(VERSION 3.5)

  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup()

else()
  find_package(catkin QUIET)
endif()

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

if (FIPS_CONFIG)

  fips_begin_module( libserdp_player )

    fips_src( lib/ )

    fips_deps( g3logger oculus serdp_common serdp_gpmf activeobject )

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
  if( NOT FIPS_IMPORT )

    if( FIPS_UNITTESTS )
      gtest_begin( serdp_common )
      fips_src( test/unit/ )
      fips_deps( serdp_common )
      gtest_end()
    endif()

  endif()
  fips_finish()

else()

  find_package(catkin REQUIRED
    roscpp
    gpmf
    g3log_catkin
    liboculus REQUIRED
    serdp_common
    serdp_gpmf
    active_object
    cv_bridge
    sensor_msgs
    image_transport
    imaging_sonar_msgs
    rosbag
  )

  catkin_package(
  	CATKIN_DEPENDS
      liboculus
      serdp_gpmf
      serdp_common
      sensor_msgs
      imaging_sonar_msgs
      rosbag
    INCLUDE_DIRS include
    LIBRARIES serdp_player_LIB
  )

  include_directories(
      include
      ros/include/
      ${catkin_INCLUDE_DIRS}
    )


  file(GLOB serdp_player_SRCS
      ros/lib/*cpp
      lib/*.cpp
    )


  add_library(serdp_player_LIB ${serdp_player_SRCS})
  target_link_libraries(serdp_player_LIB ${catkin_LIBRARIES} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVDEVICE_LIBRARY} ${SWSCALE_LIBRARIES} ${AVUTIL_LIBRARIES} ${OpenCV_LIBS} )

  add_executable(serdp_player_ros ros/tools/serdp_player_ros.cpp)
  target_link_libraries(serdp_player_ros serdp_player_LIB )


  install(TARGETS serdp_player_ros serdp_player_LIB
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

  ## Install headers
  install(DIRECTORY  include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
      PATTERN ".git" EXCLUDE)

endif()
