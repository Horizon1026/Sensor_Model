cmake_minimum_required( VERSION 3.10 )

project( sensor_model )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

find_package( OpenCV REQUIRED )

# Add slam utility
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_slam_utility )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_slam_utility )
endif()

# Add camera model.
if ( NOT TARGET lib_camera_model )
	add_subdirectory( src/camera ${PROJECT_SOURCE_DIR}/build/lib_camera_model )
endif()

add_executable( test_camera_model
    test/test_camera_model.cpp
)

target_link_libraries( test_camera_model
    ${OpenCV_LIBS}
    lib_slam_utility
    lib_camera_model
)
