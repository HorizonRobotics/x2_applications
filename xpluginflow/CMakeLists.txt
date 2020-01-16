cmake_minimum_required(VERSION 2.8)

# 项目名称
project(xpluginflow)

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
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

add_subdirectory(test)
add_subdirectory(sample)

# make install才会安装
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/xpluginflow)
install(TARGETS xpluginflow
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/xpluginflow/
        DESTINATION ${MY_OUTPUT_ROOT}/include/xpluginflow)
