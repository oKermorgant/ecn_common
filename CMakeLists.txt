cmake_minimum_required(VERSION 2.8.3)
project(ecn_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge roscpp image_transport std_msgs sensor_msgs geometry_msgs
  DEPENDS ${VISP_LIBRARIES}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

## Main C++ library
add_library(${PROJECT_NAME}
                include/ecn_common/token.h
                include/ecn_common/optim.h
                src/optim.cpp
                include/ecn_common/color_detector.h
                src/color_detector.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

## Test executables
add_executable(token_example examples/token_example.cpp)
target_link_libraries(token_example ${PROJECT_NAME})

add_executable(color_example examples/color_example.cpp)
target_link_libraries(color_example ${PROJECT_NAME})

add_executable(qp_example examples/qp_example.cpp)
target_link_libraries(qp_example ${PROJECT_NAME})

add_executable(hist examples/test_hist.cpp)
target_link_libraries(hist ${PROJECT_NAME})




#############
## Install ##
#############


catkin_install_python(PROGRAMS src/token_manager
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

 # only install actual library
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
