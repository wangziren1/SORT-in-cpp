cmake_minimum_required (VERSION 3.16)
project (sort VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Release)

add_subdirectory("thirdparty/glog-master")

set(Eigen3_DIR "${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.4.0/build")
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(sort)
add_subdirectory(MOT15)
