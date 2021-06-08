cmake_minimum_required(VERSION 3.0.2)
project(LidarLoca)


set(CMAKE_CXX_FLAGS "-std=c++11")

# 不调试的时候打开下面
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ") # -g
set (CMAKE_VERBOSE_MAKEFILE ON)

# 调试的时候打开下面
set(CMAKE_BUILD_TYPE "Release")    # 调式开关

IF (CMAKE_BUILD_TYPE STREQUAL Debug)     #   用IF控制，可以实现Release版本不输出debug信息
    ADD_DEFINITIONS(-DDEBUG)             #               不知道为什么DEBUG必须写成-DDEBUG，谁来解释一下吧
ENDIF()

IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCC)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  visualization_msgs
  cv_bridge
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(SOURCES
${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_process.cpp
    )
set(HEADERS
${CMAKE_CURRENT_SOURCE_DIR}/include/lidar_process.h
${CMAKE_CURRENT_SOURCE_DIR}/include/common.h
${CMAKE_CURRENT_SOURCE_DIR}/include/tic_toc.h
    )


set(SOURCES_ODOM
${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_odom.cpp
    )
    set(HEADERS_ODOM
${CMAKE_CURRENT_SOURCE_DIR}/include/common.h
${CMAKE_CURRENT_SOURCE_DIR}/include/tic_toc.h
${CMAKE_CURRENT_SOURCE_DIR}/include/lidar_odom.h
${CMAKE_CURRENT_SOURCE_DIR}/include/lidarFactor.hpp
    )

    set(SOURCES_MAP
${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_mapping.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_mapping_thread.cpp


    )
    set(HEADERS_MAP
${CMAKE_CURRENT_SOURCE_DIR}/include/common.h
${CMAKE_CURRENT_SOURCE_DIR}/include/tic_toc.h
${CMAKE_CURRENT_SOURCE_DIR}/include/lidar_mapping.h
${CMAKE_CURRENT_SOURCE_DIR}/include/lidarFactor.hpp
    )


include_directories(
 ${CMAKE_CURRENT_SOURCE_DIR}/include
 include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS  PCL 
  INCLUDE_DIRS include
)

add_executable(lidar_process_node    src/lidar_registration.cpp  ${SOURCES} ${HEADERS})
target_link_libraries( lidar_process_node   ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(lidar_odometry_node    src/lidar_odometry_node.cpp  ${SOURCES_ODOM} ${HEADERS_ODOM})
target_link_libraries( lidar_odometry_node   ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(lidar_mapping_node    src/lidar_mapping_node.cpp  ${SOURCES_MAP} ${HEADERS_MAP})
target_link_libraries( lidar_mapping_node   ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

