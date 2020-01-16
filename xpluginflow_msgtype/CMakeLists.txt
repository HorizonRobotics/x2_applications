cmake_minimum_required(VERSION 2.8)

project(xpluginflow_msgtype)

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
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_link_libraries(${PROJECT_NAME} xpluginflow ${BASE_LIBRARIES})
else ()
    target_link_libraries(${PROJECT_NAME} xpluginflowd ${BASE_LIBRARIES})
endif ()
# add_subdirectory(test)

# make install才会安装
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
install(TARGETS xpluginflow_msgtype
        DESTINATION ${MY_OUTPUT_ROOT}/lib/)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
