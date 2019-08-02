fips_begin_app(serdp_player cmdline)
  fips_files( serdp_player.cpp )
  fips_deps(serdpplayer)
  fips_include_directories(
      ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()
