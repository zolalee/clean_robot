cmake_minimum_required(VERSION 2.8)
project(um_path_planning)

message("=================path_planning CMAKELIST========================")
message(${PROJECT_SOURCE_DIR})
add_compile_options(-std=c++11)

#告知当前使用的是交叉编译方式，必须配置
SET(CMAKE_SYSTEM_NAME Linux)
# set (CMAKE_CXX_STANDARD 11)
#指定C交叉编译器,必须配置
#或交叉编译器使用绝对地址
SET(CMAKE_C_COMPILER "/home/c/yunshi_robot/tina-master/prebuilt/gcc/linux-x86/arm/toolchain-sunxi-musl/toolchain/bin/arm-openwrt-linux-muslgnueabi-gcc-6.4.1")

#指定C++交叉编译器
SET(CMAKE_CXX_COMPILER "/home/c/yunshi_robot/tina-master/prebuilt/gcc/linux-x86/arm/toolchain-sunxi-musl/toolchain/bin/arm-openwrt-linux-muslgnueabi-g++")

set(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(INCLUDE_DIR
 ../include
 ../include/common_fucntion
 ../include/navigation_algorithm
 ../include/um_chassis
 ../include/ros_lib
 ../include/message
 ../include/msgpack
 ../include/rpc_core
 ../include/boost
)

set(LIB_DIR 
  lib
  ${PROJECT_SOURCE_DIR}/lib/arm 
)


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# include_directories(../include)
include_directories(${INCLUDE_DIR})
link_directories(${PROJECT_SOURCE_DIR}/lib/arm/)
link_directories(${PROJECT_SOURCE_DIR}/lib/arm/lib)
# add_library(
#         planning
#         RoadPlanning.cpp
#         LocalPlanning.cpp
#         GlobalPlanning.cpp
#         BlockPlanning.cpp
#         InternalSpiralPlanning.cpp
#         UTurnPlanning.cpp
#         MotionControl.cpp
#         MapFunction.cpp
#         chassisBase.cpp
#         Parse.cpp
#         Recharge.cpp
# )
# link_libraries("/lib/arm/liblidar_shm.a"
#               "/lib/arm/libum_chassis_user.a")

option(SIMULATION "This is a default option for PUBLIC" OFF)
message("build planning platform switch: ${SIMULATION}")
if(${SIMULATION} STREQUAL "ON")
  add_library(
          planning
          RoadPlanning.cpp
          LocalPlanning.cpp
          GlobalPlanning.cpp
          BlockPlanning.cpp
          InternalSpiralPlanning.cpp
          UTurnPlanning.cpp
          MotionControl.cpp
          MapFunction.cpp
          simulationChassisBase.cpp
          Parse.cpp
          Recharge.cpp
          ExceptionHanding.cpp
          RemoteControllerTask.cpp

  )
  add_executable(um_planning_node 
  ${PROJECT_SOURCE_DIR}/planning_node_test.cpp 
  ${PROJECT_SOURCE_DIR}/PathPlanningInterface.cpp
  ${PROJECT_SOURCE_DIR}/MotionControl.cpp
  ${PROJECT_SOURCE_DIR}/MapFunction.cpp
  ${PROJECT_SOURCE_DIR}/simulationChassisBase.cpp
  ${PROJECT_SOURCE_DIR}/GlobalPlanning.cpp
  ${PROJECT_SOURCE_DIR}/UTurnPlanning.cpp
  ${PROJECT_SOURCE_DIR}/LocalPlanning.cpp
  ${PROJECT_SOURCE_DIR}/InternalSpiralPlanning.cpp
  ${PROJECT_SOURCE_DIR}/Recharge.cpp
  ${PROJECT_SOURCE_DIR}/Parse.cpp

  )
else()
  add_library(
          planning
          RoadPlanning.cpp
          LocalPlanning.cpp
          GlobalPlanning.cpp
          BlockPlanning.cpp
          InternalSpiralPlanning.cpp
          UTurnPlanning.cpp
          MotionControl.cpp
          MapFunction.cpp
          chassisBase.cpp
          Parse.cpp
          Recharge.cpp
          ExceptionHanding.cpp
          # lz4.c
          AlongWall.cpp
  )
  add_executable(um_planning_node 
  ${PROJECT_SOURCE_DIR}/planning_node.cpp 
  ${PROJECT_SOURCE_DIR}/PathPlanningInterface.cpp
  ${PROJECT_SOURCE_DIR}/RemoteControllerTask.cpp
  ${PROJECT_SOURCE_DIR}/MotionControl.cpp
  ${PROJECT_SOURCE_DIR}/MapFunction.cpp
  ${PROJECT_SOURCE_DIR}/GlobalPlanning.cpp
  ${PROJECT_SOURCE_DIR}/BlockPlanning.cpp
  ${PROJECT_SOURCE_DIR}/UTurnPlanning.cpp
  ${PROJECT_SOURCE_DIR}/LocalPlanning.cpp
  ${PROJECT_SOURCE_DIR}/InternalSpiralPlanning.cpp
  ${PROJECT_SOURCE_DIR}/chassisBase.cpp
  ${PROJECT_SOURCE_DIR}/Recharge.cpp
  ${PROJECT_SOURCE_DIR}/Parse.cpp
  ${PROJECT_SOURCE_DIR}/RoadPlanning.cpp
  ${PROJECT_SOURCE_DIR}/ExceptionHanding.cpp
  ${PROJECT_SOURCE_DIR}/navigation_rpc_server.cpp
  ${PROJECT_SOURCE_DIR}/navigation_server_handler.cpp
  # ${PROJECT_SOURCE_DIR}/lz4.c
  ${PROJECT_SOURCE_DIR}/AlongWall.cpp
  )
endif()


# add_executable(um_planning_node 
# ${PROJECT_SOURCE_DIR}/planning_node.cpp 
# ${PROJECT_SOURCE_DIR}/PathPlanningInterface.cpp
# ${PROJECT_SOURCE_DIR}/MotionControl.cpp
# ${PROJECT_SOURCE_DIR}/MapFunction.cpp
# ${PROJECT_SOURCE_DIR}/chassisBase.cpp
# ${PROJECT_SOURCE_DIR}/GlobalPlanning.cpp
# ${PROJECT_SOURCE_DIR}/UTurnPlanning.cpp
# ${PROJECT_SOURCE_DIR}/LocalPlanning.cpp
# ${PROJECT_SOURCE_DIR}/InternalSpiralPlanning.cpp
# ${PROJECT_SOURCE_DIR}/chassisBase.cpp
# ${PROJECT_SOURCE_DIR}/Recharge.cpp
# ${PROJECT_SOURCE_DIR}/Parse.cpp
# )
TARGET_LINK_LIBRARIES (um_planning_node 
      ${PROJECT_SOURCE_DIR}/lib/arm/liblidar_shm.a
      ${PROJECT_SOURCE_DIR}/lib/arm/libum_chassis_user.a
      ${PROJECT_SOURCE_DIR}/lib/arm/libyaml-cpp.a
      core
      msgpackc-cxx
      ${PROJECT_SOURCE_DIR}/lib/arm/libcore.so      
      ${PROJECT_SOURCE_DIR}/lib/arm/libChassisProtocol.a
      ${PROJECT_SOURCE_DIR}/lib/arm/libsemaphore.a
      ${PROJECT_SOURCE_DIR}/lib/arm/liblogger.a
      ${PROJECT_SOURCE_DIR}/lib/arm/lib/

      # ${PROJECT_SOURCE_DIR}/lib/arm/libsqlite3.a
)

message("=============End of path_planning CMAKELIST=====================")




