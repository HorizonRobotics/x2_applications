cmake_minimum_required(VERSION 2.8)
project(FaceSnapFilterMethod)

message("config types: ${CMAKE_CONFIGURATION_TYPES}")

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
list(APPEND CMAKE_C_FLAGS " -O3 -march=armv8-a -mcpu=cortex-a53")
message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}" )

set(CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC")
endif()

include_directories(
        include
        src
        deps/xroc-framework/include
        deps/jsoncpp/include
        deps/hobotlog/include
        deps/vision_type/include
        deps/opencv/include
        deps/gtest/include
)

link_directories(
        ${CMAKE_SOURCE_DIR}/deps/xroc-framework/lib
        ${CMAKE_SOURCE_DIR}/deps/jsoncpp/lib
        ${CMAKE_SOURCE_DIR}/deps/hobotlog/lib
        ${CMAKE_SOURCE_DIR}/deps/vision_type/lib
        ${CMAKE_SOURCE_DIR}/deps/opencv/lib
        ${CMAKE_SOURCE_DIR}/deps/gtest/lib
)

set(SOURCE_FILES
        src/FaceSnapFilterMethod.cpp
)

add_library(FaceSnapFilterMethod STATIC ${SOURCE_FILES})

add_subdirectory(test)

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/deps/)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
