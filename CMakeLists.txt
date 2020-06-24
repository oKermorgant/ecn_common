cmake_minimum_required(VERSION 2.8.3)
project(ecn_common)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  rospy
  urdf
)

find_package(VISP REQUIRED COMPONENTS core)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Python libs
catkin_python_setup()

add_message_files(
   FILES
   TokenCurrent.msg
   TokenRequest.msg
 )
 
generate_messages()

catkin_package(
  INCLUDE_DIRS include ${VISP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy cv_bridge roscpp image_transport std_msgs
                    sensor_msgs geometry_msgs
  DEPENDS ${VISP_LIBRARIES} ${OpenCV_LIBS} 
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
                include/ecn_common/token_handle.h
                include/ecn_common/visp_utils.h
                include/ecn_common/color_detector.h
                src/color_detector.cpp
                include
                )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

## Test executables
add_executable(token_cpp_example examples/token_cpp_example.cpp)
target_link_libraries(token_cpp_example ${PROJECT_NAME})
add_dependencies(token_cpp_example ecn_common_gencpp)

add_executable(color_example examples/color_example.cpp)
target_link_libraries(color_example ${PROJECT_NAME})

add_executable(qp_example examples/qp_example.cpp)
target_link_libraries(qp_example ${PROJECT_NAME})

#add_executable(hist examples/test_hist.cpp)
#target_link_libraries(hist ${PROJECT_NAME})


#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/token_manager examples/token_python_example
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

 # only install actual library
 install(TARGETS ${PROJECT_NAME} token_cpp_example 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

  install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.user.template*"
  PATTERN ".svn" EXCLUDE)
