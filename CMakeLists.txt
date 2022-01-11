cmake_minimum_required(VERSION 2.8)
project(um_path_planning)

message("=================path_planning makefile========================")
message(${PROJECT_SOURCE_DIR})
add_compile_options(-std=c++11)

#告知当前使用的是交叉编译方式，必须配置
SET(CMAKE_SYSTEM_NAME Linux)
add_subdirectory(src)
add_subdirectory(include/msgpack)
#指定C交叉编译器,必须配置
#或交叉编译器使用绝对地址
SET(CMAKE_C_COMPILER "/home/c/yunshi_robot/tina-master/prebuilt/gcc/linux-x86/arm/toolchain-sunxi-musl/toolchain/bin/arm-openwrt-linux-muslgnueabi-gcc-6.4.1")

#指定C++交叉编译器
SET(CMAKE_CXX_COMPILER "/home/c/yunshi_robot/tina-master/prebuilt/gcc/linux-x86/arm/toolchain-sunxi-musl/toolchain/bin/arm-openwrt-linux-muslgnueabi-g++")
#set(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CONFIGURATION_TYPES Release CACHE STRING "" FORCE)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib CACHE PATH "Build directory" FORCE)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib CACHE PATH "Build directory" FORCE)


#include_directories(/include)
message("=============End of path_planning makefile=====================")




