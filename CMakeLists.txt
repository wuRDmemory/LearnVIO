cmake_minimum_required(VERSION 3.0.2)
project(learn_vio)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE RELEASE)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# OPENCV
find_package(OpenCV REQUIRED)
message("-- Found OpenCV Version: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

# EIGEN
include_directories(/usr/include/eigen3/)

# ceres
find_package(Ceres REQUIRED)
message("-- Found Ceres Version: ${CERES_VERSION}")
include_directories(${CERES_INCLUDE_DIRS})

# protobuf
find_package(Protobuf REQUIRED)
message("-- Found Protobuf version: ${PROTOBUF_VERSION}")
include_directories(${PROTOBUF_INCLUDE_DIRS})

# header
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/include/inertial/)
include_directories(${PROJECT_SOURCE_DIR}/include/visual/)
include_directories(${PROJECT_SOURCE_DIR}/include/util/)
include_directories(${PROJECT_SOURCE_DIR}/include/initial/)
include_directories(${PROJECT_SOURCE_DIR}/protobuf/c++/)
include_directories(${PROJECT_SOURCE_DIR}/camera/)

# source
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB INR_FILES ${PROJECT_SOURCE_DIR}/src/inertial/*.cc)
file(GLOB VIS_FILES ${PROJECT_SOURCE_DIR}/src/visual/*.cc)
file(GLOB UTL_FILES ${PROJECT_SOURCE_DIR}/src/util/*.cc)
file(GLOB INI_FILES ${PROJECT_SOURCE_DIR}/src/initial/*.cc)
file(GLOB PRT_FILES ${PROJECT_SOURCE_DIR}/protobuf/c++/*.cc)
file(GLOB CMR_FILES ${PROJECT_SOURCE_DIR}/camera/*.cc)

set(ALL_SOURCE 
    ${SRC_FILES} 
    ${INR_FILES} 
    ${VIS_FILES}
    ${UTL_FILES}
    ${INI_FILES}
    ${CMR_FILES}
    ${PRT_FILES})

list(APPEND ALL_LIBS
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${CERES_LIBRARIES}
    ${PROTOBUF_LIBRARIES}   
)

# compile options
add_definitions(-DUSE_ROS)

# add_executable(feature_track_node feature_track_node.cc ${ALL_SOURCE})
# target_link_libraries(feature_track_node ${ALL_LIBS})

add_executable(front_end_node front_end_node.cc ${ALL_SOURCE})
target_link_libraries(front_end_node ${ALL_LIBS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_LearnVIO.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
