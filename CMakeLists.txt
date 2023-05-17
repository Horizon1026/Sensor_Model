cmake_minimum_required( VERSION 3.10 )

project( sensor_model )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

find_package( OpenCV REQUIRED )

# Add camera model.
if ( NOT TARGET lib_camera_model )
	add_subdirectory( src/camera ${PROJECT_SOURCE_DIR}/build/lib_camera_model )
endif()

# Add imu model.
if ( NOT TARGET lib_imu_model )
	add_subdirectory( src/imu ${PROJECT_SOURCE_DIR}/build/lib_imu_model )
endif()

# Add covisible graph.
if ( NOT TARGET lib_covisible_graph )
    add_subdirectory( src/covisible_graph ${PROJECT_SOURCE_DIR}/build/lib_covisible_graph )
endif()

# Create executable target to test camera model.
add_executable( test_camera_model
    test/test_camera_model.cpp
)
target_link_libraries( test_camera_model
    ${OpenCV_LIBS}
    lib_camera_model
)

# Create executable target to test imu model.
add_executable( test_imu_model
    test/test_imu_model.cpp
)
target_link_libraries( test_imu_model
    lib_imu_model
)

# Create executable target to test covisible graph.
add_executable( test_covisible_graph
    test/test_covisible_graph.cpp
)
target_link_libraries( test_covisible_graph
    lib_covisible_graph
)
