
###
# Intrinsic image analysis and SfM_Data container initialization
###
add_executable(openMVG_main_SfMInit_ImageListing main_SfMInit_ImageListing.cpp)
target_link_libraries(openMVG_main_SfMInit_ImageListing
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    openMVG_exif
    ${STLPLUS_LIBRARY}
)

# Installation rules
set_property(TARGET openMVG_main_SfMInit_ImageListing PROPERTY FOLDER OpenMVG/software)
install(TARGETS openMVG_main_SfMInit_ImageListing DESTINATION bin/)

###
# Add executable that computes:
# - openMVG_main_ComputeFeatures: features and descriptors
# - openMVG_main_ComputeMatches: putative matches + geometric filtered matches
###

add_executable(openMVG_main_ComputeFeatures main_ComputeFeatures.cpp)
target_link_libraries(openMVG_main_ComputeFeatures
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_multiview
    openMVG_sfm
    ${STLPLUS_LIBRARY}
    vlsift
)
if (MSVC)
  set_target_properties(openMVG_main_ComputeFeatures PROPERTIES COMPILE_FLAGS "/bigobj")
endif (MSVC)


add_executable( openMVG_main_PairGenerator main_PairGenerator.cpp )
target_link_libraries( openMVG_main_PairGenerator
  PRIVATE
    openMVG_features
    openMVG_multiview
    openMVG_sfm
    openMVG_system
    ${STLPLUS_LIBRARY}
)

add_executable(openMVG_main_ComputeMatches main_ComputeMatches.cpp)
target_link_libraries(openMVG_main_ComputeMatches
  PRIVATE
    openMVG_graph
    openMVG_features
    openMVG_matching_image_collection
    openMVG_multiview
    openMVG_sfm
    openMVG_system
    ${STLPLUS_LIBRARY}
)

add_executable(openMVG_main_GeometricFilter main_GeometricFilter.cpp)
target_link_libraries(openMVG_main_GeometricFilter
  PRIVATE
    openMVG_graph
    openMVG_features
    openMVG_matching_image_collection
    openMVG_multiview
    openMVG_sfm
    openMVG_system
    ${STLPLUS_LIBRARY}
)

# Installation rules
set_property( TARGET openMVG_main_ComputeFeatures   PROPERTY FOLDER OpenMVG/software )
set_property( TARGET openMVG_main_PairGenerator     PROPERTY FOLDER OpenMVG/software )
set_property( TARGET openMVG_main_ComputeMatches    PROPERTY FOLDER OpenMVG/software )
set_property( TARGET openMVG_main_GeometricFilter   PROPERTY FOLDER OpenMVG/software )

install( TARGETS openMVG_main_ComputeFeatures   DESTINATION bin/ )
install( TARGETS openMVG_main_PairGenerator     DESTINATION bin/ )
install( TARGETS openMVG_main_ComputeMatches    DESTINATION bin/ )
install( TARGETS openMVG_main_GeometricFilter   DESTINATION bin/ )

###
# SfM Pipelines & SfM Data format tools, ...
###
add_executable(openMVG_main_SfM main_SfM.cpp)
target_link_libraries(openMVG_main_SfM
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)

add_executable(openMVG_main_ComputeStructureFromKnownPoses main_ComputeStructureFromKnownPoses.cpp)
target_include_directories(openMVG_main_ComputeStructureFromKnownPoses
  PRIVATE
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(openMVG_main_ComputeStructureFromKnownPoses
  PRIVATE
    openMVG_system
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
    ${CERES_LIBRARIES}
)

add_executable(openMVG_main_ComputeSfM_DataColor main_ComputeSfM_DataColor.cpp)
target_link_libraries(openMVG_main_ComputeSfM_DataColor
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)

# Installation rules
set_property(TARGET openMVG_main_SfM PROPERTY FOLDER OpenMVG/software)
install(TARGETS openMVG_main_SfM DESTINATION bin/)
set_property(TARGET openMVG_main_ComputeStructureFromKnownPoses PROPERTY FOLDER OpenMVG/software)
install(TARGETS openMVG_main_ComputeStructureFromKnownPoses DESTINATION bin/)
set_property(TARGET openMVG_main_ComputeSfM_DataColor PROPERTY FOLDER OpenMVG/software)
install(TARGETS openMVG_main_ComputeSfM_DataColor DESTINATION bin/)

##
# Export tutorial with valid path thanks to a configuration file
##
set(OPENMVG_SOFTWARE_SFM_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})
if(MSVC)
  set(OPENMVG_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
else()
  if( APPLE )
    if( CMAKE_GENERATOR MATCHES "Xcode" )
      set(OPENMVG_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
    else()
      set(OPENMVG_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}")
    endif()
  else()
    # Linux
    set(OPENMVG_SOFTWARE_SFM_BUILD_DIR ${EXECUTABLE_OUTPUT_PATH})
  endif()
endif()

set(OPENMVG_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_SequentialPipeline.py.in")
configure_file(${OPENMVG_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_SequentialPipeline.py" @ONLY)

add_subdirectory(export)
add_subdirectory(import)
