cmake_minimum_required(VERSION 2.8)

project(xpluginflow_msgtype)

set(CMAKE_CXX_STANDARD 11)
option(RELEASE_LIB "build version of release" ON)

# 编译模式
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
add_definitions(-DHR_POSIX)
add_definitions(-DHR_LINUX)
add_definitions(-DX2)
list(APPEND CMAKE_C_FLAGS "-O3 -march=armv8-a -mcpu=cortex-a53")

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC -Ofast ")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DEBUG -g -O0 -fPIC ")
endif ()


list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

# 添加头文件路径
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/xpluginflow_msgtype/protobuf
        ${PROJECT_SOURCE_DIR}/deps/opencv/include
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/include
        ${PROJECT_SOURCE_DIR}/deps/protobuf/include
        ${PROJECT_SOURCE_DIR}/deps/x2_prebuilt/include
        ${PROJECT_SOURCE_DIR}/deps/gtest/include
        ${PROJECT_SOURCE_DIR}/deps/vision_type/include
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow/include
        )
# 添加链接库路径
link_directories(
    ${PROJECT_SOURCE_DIR}/deps/opencv/lib
    ${PROJECT_SOURCE_DIR}/deps/hobotlog/lib
    ${PROJECT_SOURCE_DIR}/deps/protobuf/lib
    ${PROJECT_SOURCE_DIR}/deps/x2_prebuilt/lib
    ${PROJECT_SOURCE_DIR}/deps/gtest/lib
    ${PROJECT_SOURCE_DIR}/deps/vision_type/lib
    ${PROJECT_SOURCE_DIR}/deps/xpluginflow/lib
)
# 头文件路径信息
file(GLOB PROJECT_INC
        "include/*.h"
        "include/xpluginflow_msgtype/*.h"
        "include/xpluginflow_msgtype/protobuf/*.h"
        )

# 源文件路径信息
file(GLOB PROJECT_SRC
        "src/*.cc"
        "src/protobuf/*.cc"
        )

set(SOURCE_FILES
        ${PROJECT_SRC}
        ${PROJECT_INC}
        )

set(BASE_LIBRARIES ${BASE_LIBRARIES} hobotlog vision_type_util protobuf ${ARCH_LIBS})

set(OpenCVLib opencv_world)
set(OpenCVDeps libjasper libjpeg-turbo libpng libprotobuf libtiff libwebp quirc tegra_hal zlib)
set(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})
add_library(xpluginflow_msgtype STATIC ${SOURCE_FILES})

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_link_libraries(${CMAKE_PROJECT_NAME} xpluginflow ${BASE_LIBRARIES})
else ()
    target_link_libraries(${CMAKE_PROJECT_NAME} xpluginflowd ${BASE_LIBRARIES})
endif ()
# add_subdirectory(test)

# make install才会安装
set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/deps)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${CMAKE_PROJECT_NAME})
install(TARGETS xpluginflow_msgtype
        DESTINATION ${MY_OUTPUT_ROOT}/lib/)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${CMAKE_PROJECT_NAME})
