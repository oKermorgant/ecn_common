cmake_minimum_required(VERSION 2.8.3)
project(ecn_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ecn_common
  CATKIN_DEPENDS cv_bridge roscpp image_transport
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Main C++ library
add_library(${PROJECT_NAME}
                include/ecn_common/token.h
                include/ecn_common/color_detector.h
                src/color_detector.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Test executables
add_executable(token_example examples/token_example.cpp)
target_link_libraries(token_example ${PROJECT_NAME})

add_executable(color_example examples/color_example.cpp)
target_link_libraries(color_example ${PROJECT_NAME})

