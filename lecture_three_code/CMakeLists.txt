cmake_minimum_required(VERSION 2.8)
project(VSLAM_CH3)

set(CMAKE_BUILD_TYPE "Release")

# check C++11 or C++0x support
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${EIGEN3_SOURCE_DIR}
        ${Pangolin_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
)


add_executable(trajectorydrawing draw_trajectory.cpp )
target_link_libraries(trajectorydrawing ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(trajectory_coordinate draw_trajectory_coordinate.cpp )
target_link_libraries(trajectory_coordinate ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(pango_base_01 pangolin_base_01.cpp )
target_link_libraries(pango_base_01 ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(trajector_error trajector_error.cpp)
target_link_libraries(trajector_error ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(pangolin_base_02 pangolin_base_02.cpp)
target_link_libraries(pangolin_base_02 ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

#add_executable(pangolin_base_03 pangolin_base_03.cpp)
#target_link_libraries(pangolin_base_03 ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})