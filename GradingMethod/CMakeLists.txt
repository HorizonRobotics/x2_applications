cmake_minimum_required(VERSION 2.8)

project(GradingMethod)

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
        deps/xroc-framework/include
        deps/jsoncpp/include
        deps/hobotlog/include
        deps/opencv/include
        deps/vision_type/include
        deps/gtest/include
)

link_directories(
    ${CMAKE_SOURCE_DIR}/deps/xroc-framework/lib
    ${CMAKE_SOURCE_DIR}/deps/jsoncpp/lib
    ${CMAKE_SOURCE_DIR}/deps/hobotlog/lib
    ${CMAKE_SOURCE_DIR}/deps/opencv/lib
    ${CMAKE_SOURCE_DIR}/deps/vision_type/lib
    ${CMAKE_SOURCE_DIR}/deps/gtest/lib
)

set(BASE_LIBRARIES xroc-framework hobotlog jsoncpp.a rt dl pthread)

set(SOURCE_FILES
        src/method/GradingMethod.cpp
        src/method/WeightGrading.cpp
)

set(GRADING_LIBRARIES rt)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${GRADING_LIBRARIES}  ${BASE_LIBRARIES} )

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/deps/)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/GradingMethod.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

add_subdirectory(test)
