cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories(/usr/inlcude/eigen3/)

# Ceres
find_package(Ceres REQUIRED)
message("-- Ceres Version: ${CERES_VERSION}")
include_directories(${CERES_INCLUDE_DIRS})

# header
include_directories(/home/ubuntu/catkin_ws/src/learn_vio/include/inertial/)
include_directories(/home/ubuntu/catkin_ws/src/learn_vio/include/util/)
include_directories(/home/ubuntu/catkin_ws/src/learn_vio/include/visual/)

# source
file(GLOB INR_SRC /home/ubuntu/catkin_ws/src/learn_vio/src/inertial/*.cc)
file(GLOB VIS_SRC /home/ubuntu/catkin_ws/src/learn_vio/src/visual/*.cc)
file(GLOB UTL_SRC /home/ubuntu/catkin_ws/src/learn_vio/src/util/*.cc)

add_executable(test_preintegrate test_preintegrate.cc ${INR_SRC} ${UTL_SRC})
target_link_libraries(test_preintegrate ${OpenCV_LIBS})

add_executable(test_visual_factor test_visual_factor.cc ${VIS_SRC} ${UTL_SRC})
target_link_libraries(test_visual_factor ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(test_quaternion test_quaternion.cc ${INR_SRC} ${UTL_SRC})
target_link_libraries(test_quaternion ${OpenCV_LIBS})