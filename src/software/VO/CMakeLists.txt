
add_subdirectory( AlternativeVO )

# - VO (WIP)
#
add_executable(openMVG_main_VO main_VO.cpp Monocular_VO.hpp CGlWindow.hpp)
target_link_libraries(openMVG_main_VO
  openMVG_sfm
  ${STLPLUS_LIBRARY}
  openMVG_image
)
set_property(TARGET openMVG_main_VO PROPERTY FOLDER OpenMVG/software)
