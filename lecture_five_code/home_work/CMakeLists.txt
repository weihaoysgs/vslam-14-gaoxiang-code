cmake_minimum_required(VERSION 3.1.0)
project(VSLAM_LECTRUE_FOUR)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package( OpenCV REQUIRED )
find_package( Pangolin REQUIRED )
include_directories( 
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_SOURCE_DIR}
)

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image 
    ${OpenCV_LIBRARIES}
)

add_executable(disparity disparity.cpp)
target_link_libraries(disparity 
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)