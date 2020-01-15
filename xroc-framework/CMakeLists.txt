cmake_minimum_required(VERSION 2.8)
project(xroc-framework)

option(RELEASE_LIB "build version of release" ON)

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
list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")
message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}" )

set(CMAKE_CXX_STANDARD 11)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message("building release")
    list(APPEND CMAKE_C_FLAGS " -O3")
else ()
    list(APPEND CMAKE_C_FLAGS " -Og -g ")
endif()

list(APPEND CMAKE_C_FLAGS " -Wall -fPIC")
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})


include_directories(
        include
        include/common
        include/log
        include/timer
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/include
        ${PROJECT_SOURCE_DIR}/deps/vision_type/include
        ${PROJECT_SOURCE_DIR}/deps/jsoncpp/include
        ${PROJECT_SOURCE_DIR}/deps/gtest/include
        ${PROJECT_SOURCE_DIR}/deps/opencv/include
)
link_directories(
    ${PROJECT_SOURCE_DIR}/deps/hobotlog/lib
    ${PROJECT_SOURCE_DIR}/deps/vision_type/lib
    ${PROJECT_SOURCE_DIR}/deps/jsoncpp/lib
    ${PROJECT_SOURCE_DIR}/deps/gtest/lib
    ${PROJECT_SOURCE_DIR}/deps/opencv/lib
)

message("add src files ...")
set(SOURCE_FILES
        src/common/com_func.cpp
        src/profiler.cpp
        src/timer/timer.cpp
        src/method_manager.cpp
        src/node.cpp
        src/scheduler.cpp
        src/xroc.cpp
        src/xroc_data.cpp
        src/thread_pool.cpp
        src/xroc_config.cpp
        src/xroc_capi.cpp
        src/xroc_capi_type.cpp
        src/xroc_c_api.cpp
        src/method.cpp
        )

add_library(xroc-framework STATIC ${SOURCE_FILES})

add_subdirectory(example/bbox_filter)

add_subdirectory(test)


set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/deps/)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/hobotxsdk
        DESTINATION ${MY_OUTPUT_ROOT}/include/
        FILES_MATCHING PATTERN "*.h")
install(FILES
        ${PROJECT_SOURCE_DIR}/include/hobotxroc/method.h
        ${PROJECT_SOURCE_DIR}/include/hobotxroc/method_factory.h
        ${PROJECT_SOURCE_DIR}/include/hobotxroc/profiler.h
        ${PROJECT_SOURCE_DIR}/include/hobotxroc/xroc_config.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/hobotxroc)

install(TARGETS     ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

