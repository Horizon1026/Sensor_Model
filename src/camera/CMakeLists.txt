aux_source_directory( . AUX_SRC_CAMERA_MODEL )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()

if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()

add_library( lib_camera_model ${AUX_SRC_CAMERA_MODEL} )
target_include_directories( lib_camera_model PUBLIC . )
target_link_libraries( lib_camera_model
    lib_slam_utility_datatype
    lib_slam_utility_operate
)
