cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(lihash_slam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Setting build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Setting the flags for profiling information or not
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Setting Release options")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -O3 -g")
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Setting Debug options")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O1 -pg")
endif()

# Check C++ support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   message(STATUS "Using flag -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   message(STATUS "Using flag -std=c++11")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14/11 support.
                        Please use a different C++ compiler.")
endif()

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  eigen_conversions
  pcl_conversions
  tf_conversions  
  message_generation
  image_transport
  cv_bridge
  cmake_modules
)

# Other packages
find_package(Eigen3)            # 3.3.7
find_package(PCL REQUIRED)      # 1.10
find_package(Ceres REQUIRED)    # 2.0.0
find_package(G2O REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Printing the compiling flags
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  LoopClosure.msg
  KeyframeMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf eigen_conversions pcl_conversions tf_conversions message_runtime image_transport cv_bridge
  DEPENDS EIGEN3 PCL CERES G2O
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
)

#### Preprocessor Node
add_executable(preprocessor
    src/pc_processor.cc
    src/nodes/preprocessor.cc)
target_link_libraries(preprocessor
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#### Odometer Node
add_executable(odometer
    src/lidar_odometry.cc
    src/nodes/odometer.cc)
target_link_libraries(odometer
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
)

#### Loop Detector Node
add_executable(loop_detector
    include/lihash_slam/loop_detector_base.hpp
    src/loop_detector_pose.cc
    src/iscGenerationClass.cpp
    src/Scancontext.cpp
    src/loop_detector_isc.cc
    src/loop_detector_sc.cc
    src/nodes/loop_detector.cc)
target_link_libraries(loop_detector
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}    
)

#### Mapper Node
add_executable(mapper    
    src/cell.cc
    src/keyframe.cc
    src/information_matrix.cpp
    src/map.cc
    src/nodes/mapper.cc)
target_link_libraries(mapper
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}    
)