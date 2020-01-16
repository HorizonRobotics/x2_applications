cmake_minimum_required(VERSION 2.8)
project(FasterRCNNMethod)

option(DUMP_RESULT "whether to dump results" OFF)
if (${DUMP_RESULT})
    add_definitions(-DDUMP)
    message("dump results..")
endif ()

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++11 -fPIC -O3 ")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++11 -DEBUG -g -Og -fPIC ")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

include_directories(
        include
        include/FasterRCNNMethod
)

set(SOURCE_FILES
        src/faster_rcnn.cpp
        src/faster_rcnn_imp.cpp
        src/yuv_utils.cc
        src/dump.cpp
        )

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

set(DYNAMIC_DEPS_LIBS bpu_predict cnn_intf hbrt_bernoulli_aarch64 cam vio fb)

#set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/deps/)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${PROJECT_SOURCE_DIR}/include/FasterRCNNMethod/FasterRCNNMethod.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/FasterRCNNMethod)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/configs
        DESTINATION ${MY_OUTPUT_ROOT}/)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

#add_subdirectory(example)
#add_subdirectory(test)
