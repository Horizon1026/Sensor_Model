cmake_minimum_required( VERSION 3.0 )

project( sensor_model )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 20 )

# Add visualizor.
set( VISUALIZOR_PATH ${PROJECT_SOURCE_DIR}/../Visualizor2D )
if ( NOT TARGET lib_2d_visualizor )
    add_subdirectory( ${VISUALIZOR_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_2d_visualizor )
endif()
set( VISUALIZOR_PATH ${PROJECT_SOURCE_DIR}/../Visualizor3D )
if ( NOT TARGET lib_3d_visualizor )
    add_subdirectory( ${VISUALIZOR_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_3d_visualizor )
endif()

# Add feature point/line detector and feature descriptor.
set( FEATURE_DETECTOR_PATH ${PROJECT_SOURCE_DIR}/../Feature_Detector )
if ( NOT TARGET lib_feature_point_detector )
add_subdirectory( ${FEATURE_DETECTOR_PATH}/src/feature_point_detector ${PROJECT_SOURCE_DIR}/build/lib_feature_point_detector )
endif()

# Add camera model.
if ( NOT TARGET lib_camera_model )
    add_subdirectory( src/camera ${PROJECT_SOURCE_DIR}/build/lib_camera_model )
endif()

# Add imu model.
if ( NOT TARGET lib_imu_model )
    add_subdirectory( src/imu ${PROJECT_SOURCE_DIR}/build/lib_imu_model )
endif()

# Add gnss model.
if ( NOT TARGET lib_gnss_model )
    add_subdirectory( src/gnss ${PROJECT_SOURCE_DIR}/build/lib_gnss_model )
endif()

# Add wheel odom model.
if ( NOT TARGET lib_wheel_odom_model )
    add_subdirectory( src/wheel_odom ${PROJECT_SOURCE_DIR}/build/lib_wheel_odom_model )
endif()

# Create executable target to test camera model.
add_executable( test_camera_model
    test/test_camera_model.cpp
)
target_link_libraries( test_camera_model
    lib_2d_visualizor
    lib_feature_point_detector
    lib_camera_model
)

# Create executable target to test imu model.
add_executable( test_imu_model
    test/test_imu_model.cpp
)
include_directories(test)
target_link_libraries( test_imu_model
    lib_imu_model
)

# Create executable target to test gnss model.
add_executable( test_gnss_model
    test/test_gnss_model.cpp
)
include_directories(test)
target_link_libraries( test_gnss_model
    lib_gnss_model
    lib_3d_visualizor
)

# Create executable target to test wheel odom model.
add_executable( test_wheel_odom_model
    test/test_wheel_odom_model.cpp
)
include_directories(test)
target_link_libraries( test_wheel_odom_model
    lib_wheel_odom_model
    lib_3d_visualizor
)
