# - Export a SfM openMVG scene to openMVS format
#
add_executable(openMVG_main_openMVG2openMVS main_openMVG2openMVS.cpp)
target_link_libraries(openMVG_main_openMVG2openMVS
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  ${STLPLUS_LIBRARY}
)

# installation rules
set_property(TARGET openMVG_main_openMVG2openMVS PROPERTY FOLDER OpenMVG/software)
install(TARGETS openMVG_main_openMVG2openMVS DESTINATION bin/)
