cmake_minimum_required(VERSION 2.8)

# 项目名称
project(xpluginflow)
option(RELEASE_LIB "build version of release" ON)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
add_definitions(-DHR_POSIX)
add_definitions(-DHR_LINUX)
add_definitions(-DX2)
list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")

# 编译模式
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()

# 编译参数

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Ofast ")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC ")
endif ()


list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

# 添加头文件路径
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/include
        ${PROJECT_SOURCE_DIR}/deps/gtest/include
)
link_directories(
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/lib
        ${PROJECT_SOURCE_DIR}/deps/gtest/lib
)
# 头文件路径信息
file(GLOB PROJECT_INC
        "include/*.h"
        "include/*.hpp"
        "include/xpluginflow/*.h"
        "include/xpluginflow/utils/*.h"
        "include/xpluginflow/message/protobuf/*.h"
        "include/xpluginflow/manager/*.h"
        "include/xpluginflow/threads/*.h"
        "include/xpluginflow/message/pluginflow/*.h"
        "include/xpluginflow/plugin/*.h"
        )

# 源文件路径信息
file(GLOB PROJECT_SRC
        "src/xpluginflow/*.cpp"
        "src/utils/*.cpp"
        "src/message/protobuf/*.cc"
        "src/threads/*.cpp"
        "src/plugin/*.cpp"
        )

set(SOURCE_FILES
        ${PROJECT_SRC}
        ${PROJECT_INC}
        )
# add_library的时候不需要target_link_library
add_library(${CMAKE_PROJECT_NAME} SHARED ${SOURCE_FILES})

add_subdirectory(test)
add_subdirectory(sample)

# make install才会安装
set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/deps)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/xpluginflow)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/build/libxpluginflow.so
        DESTINATION ${MY_OUTPUT_ROOT}/lib/)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/xpluginflow/
        DESTINATION ${MY_OUTPUT_ROOT}/include/xpluginflow)
