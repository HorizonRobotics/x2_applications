cmake_minimum_required(VERSION 2.8)

# 项目名称
project(vioplugin)

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
)
# 头文件路径信息
file(GLOB PROJECT_INC
        "include/*.h"
        "include/vioplugin/*.h"
        "include/vioplugin/utils/*.h"
        )

# 源文件路径信息
file(GLOB PROJECT_SRC
        "src/vioplugin/*.cpp"
        "src/utils/*.cpp"
        )

set(SOURCE_FILES
        ${PROJECT_SRC}
        ${PROJECT_INC}
        )
# add_library的时候不需要target_link_library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

set(OpenCVLib opencv_world)
set(OpenCVDeps turbojpeg protobuf)
set(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_link_libraries(${PROJECT_NAME} xpluginflow_msgtype xpluginflow ${BASE_LIBRARIES})
else ()
    target_link_libraries(${PROJECT_NAME} xpluginflow_msgtyped xpluginflowd ${BASE_LIBRARIES})
endif ()
#add_subdirectory(test)

# make install才会安装
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
install(TARGETS vioplugin
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/configs/
        DESTINATION ${MY_OUTPUT_ROOT}/config/)
