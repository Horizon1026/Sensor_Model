aux_source_directory( . AUX_SRC_VISUAL_MEASURE )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../Slam_Utility )
if( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${CMAKE_CURRENT_BINARY_DIR}/lib_slam_utility_basic_type )
endif()

if( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/operate ${CMAKE_CURRENT_BINARY_DIR}/lib_slam_utility_operate )
endif()

add_library( lib_visual_measure ${AUX_SRC_VISUAL_MEASURE} )
target_include_directories( lib_visual_measure PUBLIC . )
target_link_libraries( lib_visual_measure
    lib_slam_utility_basic_type
    lib_slam_utility_operate
)
