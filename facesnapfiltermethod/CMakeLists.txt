cmake_minimum_required(VERSION 2.8)
project(FaceSnapFilterMethod)

message("config types: ${CMAKE_CONFIGURATION_TYPES}")

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC")
endif()

include_directories(
        include
        src
)

set(SOURCE_FILES
        src/FaceSnapFilterMethod.cpp
)

add_library(FaceSnapFilterMethod STATIC ${SOURCE_FILES})

add_subdirectory(test)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
