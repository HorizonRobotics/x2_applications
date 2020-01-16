cmake_minimum_required(VERSION 2.8)
project(CNNMethod)

message("config types: ${CMAKE_CONFIGURATION_TYPES}")

option(USE_BGR2NV12 "select BGR to convert directly to nv12" OFF)
message("USE_BGR2NV12 is ${USE_BGR2NV12}")
if(USE_BGR2NV12)
    add_definitions(-DUSE_BGR2NV12)
    message("select BGR to convert directly to nv12")
endif(USE_BGR2NV12)

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC")
endif()
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    )
add_library(CNNMethod STATIC ${SOURCE_FILES})

add_subdirectory(test)
add_subdirectory(example)

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
