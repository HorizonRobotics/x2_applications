cmake_minimum_required(VERSION 2.8)
# 项目名称
project(stage1)
# set(CMAKE_CXX_STANDARD 11)
option(RELEASE_LIB "build version of release" ON)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")

# 编译模式
if(${RELEASE_LIB})
  set(CMAKE_BUILD_TYPE Release)
else()
  set(CMAKE_BUILD_TYPE Debug)
endif()

# 编译参数
if(NOT PLATFORM_WIN)
  if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Ofast ")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC ")
  endif()
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

# 添加头文件路径
include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/deps/hobotlog/include
  ${CMAKE_SOURCE_DIR}/deps/jsoncpp/include
  ${CMAKE_SOURCE_DIR}/deps/xroc-framework/include)

link_directories(
  ${CMAKE_SOURCE_DIR}/deps/hobotlog/lib
  ${CMAKE_SOURCE_DIR}/deps/xroc-framework/lib
  ${CMAKE_SOURCE_DIR}/deps/jsoncpp/lib)

set(BASE_LIBRARIES xroc-framework hobotlog jsoncpp pthread)

set(SOURCE_FILES ${CMAKE_SOURCE_DIR}/method/b_box_filter.cc
                 ${CMAKE_SOURCE_DIR}/method_factory.cc)

add_executable(sync_main ${CMAKE_SOURCE_DIR}/sync_main.cc ${SOURCE_FILES})
target_link_libraries(sync_main ${BASE_LIBRARIES})

add_executable(async_main ${CMAKE_SOURCE_DIR}/async_main.cc ${SOURCE_FILES})
target_link_libraries(async_main ${BASE_LIBRARIES})

add_executable(update_param_main ${CMAKE_SOURCE_DIR}/update_param_main.cc
                                 ${SOURCE_FILES})
target_link_libraries(update_param_main ${BASE_LIBRARIES})
