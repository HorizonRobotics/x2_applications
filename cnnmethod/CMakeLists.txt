cmake_minimum_required(VERSION 2.8)
project(CNNMethod)

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
list(APPEND CMAKE_C_FLAGS "-O3 -march=armv8-a -mcpu=cortex-a53")

option(USE_BGR2NV12 "select BGR to convert directly to nv12" OFF)
message("USE_BGR2NV12 is ${USE_BGR2NV12}")
if(USE_BGR2NV12)
    add_definitions(-DUSE_BGR2NV12)
    message("select BGR to convert directly to nv12")
endif(USE_BGR2NV12)

option(COVERAGE_TEST "open coverage build flag" OFF)
message("COVERAGE_TEST is ${COVERAGE_TEST}")
if (${COVERAGE_TEST})
  list(APPEND CMAKE_C_FLAGS " --coverage")
endif()
if ((NOT (${RELEASE_LIB})) OR (${COVERAGE_TEST}))
  list(APPEND CMAKE_C_FLAGS " -g -0g")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC")
endif()

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    deps/xroc-framework/include
    deps/bpu_predict/include
    deps/jsoncpp/include
    deps/hobotlog/include
    deps/opencv/include
    deps/x2_prebuilt/include
    deps/vision_type/include
    deps/libjpeg-turbo/include/
    deps/libyuv/include
    deps/xroc-imagetools/include
    deps/gtest/include
    deps/FasterRCNNMethod/include
    )

link_directories(
    ${CMAKE_SOURCE_DIR}/deps/xroc-framework/lib
    ${CMAKE_SOURCE_DIR}/deps/bpu_predict/lib
    ${CMAKE_SOURCE_DIR}/deps/jsoncpp/lib
    ${CMAKE_SOURCE_DIR}/deps/hobotlog/lib
    ${CMAKE_SOURCE_DIR}/deps/opencv/lib
    ${CMAKE_SOURCE_DIR}/deps/x2_prebuilt/lib
    ${CMAKE_SOURCE_DIR}/deps/vision_type/lib
    ${CMAKE_SOURCE_DIR}/deps/libjpeg-turbo/lib
    ${CMAKE_SOURCE_DIR}/deps/libyuv/lib
    ${CMAKE_SOURCE_DIR}/deps/xroc-imagetools/lib
    ${CMAKE_SOURCE_DIR}/deps/gtest/lib
    ${CMAKE_SOURCE_DIR}/deps/FasterRCNNMethod/lib
)
add_library(CNNMethod STATIC ${SOURCE_FILES})

add_subdirectory(test)
add_subdirectory(example)

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/deps)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
message("MY_OUTPUT_ROOT: ${MY_OUTPUT_ROOT}")
install(TARGETS CNNMethod
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(FILES ${PROJECT_SOURCE_DIR}/include/CNNMethod/CNNMethod.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/CNNMethod/)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/example/config/method_conf/
        DESTINATION ${MY_OUTPUT_ROOT}/config
        FILES_MATCHING PATTERN "nir*.json")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/example/config/method_conf/
        DESTINATION ${MY_OUTPUT_ROOT}/config
        FILES_MATCHING PATTERN "anti_spf*.json")
install(FILES ${PROJECT_SOURCE_DIR}/example/config/method_conf/pose_lmk.json
        DESTINATION ${MY_OUTPUT_ROOT}/config)
install(FILES ${PROJECT_SOURCE_DIR}/example/config/method_conf/feature.json
        DESTINATION ${MY_OUTPUT_ROOT}/config)
install(FILES ${PROJECT_SOURCE_DIR}/example/config/method_conf/face_quality.json
        DESTINATION ${MY_OUTPUT_ROOT}/config)
