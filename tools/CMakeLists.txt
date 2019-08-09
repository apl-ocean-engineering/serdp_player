fips_begin_app(serdp_player cmdline)
  fips_files( serdp_player.cpp )
  fips_deps(serdpplayer)

fips_end_app()


fips_begin_app(mov_player cmdline)
  fips_files( mov_player.cpp GPMF_print.cpp)
  fips_deps(gpmf oculus serdpplayer serdp_common activeobject)
fips_include_directories(
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${OpenCV_INCLUDE_DIRS}
      ${AVCODEC_INCLUDE_DIR}
      ${SWSCALE_INCLUDE_DIR}
      ${AVFORMAT_INCLUDE_DIR}
      #${MEM_INCLUDE_DIR}
  )
  fips_libs(
    ${OpenCV_LIBS}
    ${AVCODEC_LIBRARY}
    ${SWSCALE_LIBRARIES}
    ${AVFORMAT_LIBRARY}
    #${MEM_LIBRARIES}
  )

fips_end_app()

