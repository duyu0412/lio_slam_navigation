cmake_minimum_required(VERSION 3.10)
project(loop_closure_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED COMPONENTS common io registration filters visualization)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(loop_closure_test loop_closure_test.cpp)

target_link_libraries(loop_closure_test
    ${PCL_LIBRARIES}
)

message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
