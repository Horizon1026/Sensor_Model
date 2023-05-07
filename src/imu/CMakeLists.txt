aux_source_directory( . AUX_SRC_IMU_MODEL )

# Add all components of slam utility.
if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()

add_library( lib_imu_model ${AUX_SRC_IMU_MODEL} )
target_include_directories( lib_imu_model PUBLIC . )
target_link_libraries( lib_imu_model
    lib_slam_utility_datatype
    lib_slam_utility_operate
    lib_slam_utility_math
)
