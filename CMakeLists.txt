cmake_minimum_required(VERSION 2.8)
# 项目名称
project(xpp_sample)
option(RELEASE_LIB "build version of release" ON)

add_definitions(-DHR_POSIX)
add_definitions(-DHR_LINUX)
add_definitions(-DX2)
list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")

# 编译模式
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()

SET(DEBUG_POSTFIX "d")
set(CMAKE_DEBUG_POSTFIX ${DEBUG_POSTFIX})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
link_directories(${CMAKE_BINARY_DIR}/lib)

# 添加头文件路径
include_directories(
        ${PROJECT_SOURCE_DIR}/xroc-framework/include
        ${PROJECT_SOURCE_DIR}/fasterrcnnmethod/include
        ${PROJECT_SOURCE_DIR}/cnnmethod/include
        ${PROJECT_SOURCE_DIR}/facesnapfiltermethod/include
        ${PROJECT_SOURCE_DIR}/GradingMethod/include
        ${PROJECT_SOURCE_DIR}/SnapshotMethod/include
        ${PROJECT_SOURCE_DIR}/xpluginflow/include
        ${PROJECT_SOURCE_DIR}/xpluginflow_msgtype/include
        ${PROJECT_SOURCE_DIR}/vioplugin/include
        ${PROJECT_SOURCE_DIR}/hbipcplugin/include
        ${PROJECT_SOURCE_DIR}/smartplugin/include
        ${PROJECT_SOURCE_DIR}/external/hobotlog/include
        ${PROJECT_SOURCE_DIR}/external/vision_type/include
        ${PROJECT_SOURCE_DIR}/external/opencv/include
        ${PROJECT_SOURCE_DIR}/external/MOTMethod/include
        ${PROJECT_SOURCE_DIR}/external/protobuf/include
        ${PROJECT_SOURCE_DIR}/external/x2_prebuilt/include
        ${PROJECT_SOURCE_DIR}/external/jsoncpp/include
        ${PROJECT_SOURCE_DIR}/external/ipc_tracking/include
        ${PROJECT_SOURCE_DIR}/external/bpu_predict/include
        ${PROJECT_SOURCE_DIR}/external/gtest/include
        ${PROJECT_SOURCE_DIR}/external/hbipc/include
        ${PROJECT_SOURCE_DIR}/external/libjpeg-turbo/include
        ${PROJECT_SOURCE_DIR}/external/libyuv/include
        ${PROJECT_SOURCE_DIR}/external/xroc-imagetools/include
    )
link_directories(
    ${PROJECT_SOURCE_DIR}/external/hobotlog/lib
    ${PROJECT_SOURCE_DIR}/external/vision_type/lib
    ${PROJECT_SOURCE_DIR}/external/opencv/lib
    ${PROJECT_SOURCE_DIR}/external/MOTMethod/lib
    ${PROJECT_SOURCE_DIR}/external/protobuf/lib
    ${PROJECT_SOURCE_DIR}/external/x2_prebuilt/lib
    ${PROJECT_SOURCE_DIR}/external/jsoncpp/lib
    ${PROJECT_SOURCE_DIR}/external/ipc_tracking/lib
    ${PROJECT_SOURCE_DIR}/external/bpu_predict/lib
    ${PROJECT_SOURCE_DIR}/external/gtest/lib
    ${PROJECT_SOURCE_DIR}/external/hbipc/lib
    ${PROJECT_SOURCE_DIR}/external/libjpeg-turbo/lib
    ${PROJECT_SOURCE_DIR}/external/libyuv/lib
    ${PROJECT_SOURCE_DIR}/external/xroc-imagetools/lib
)

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/${PROJECT_NAME}/)
message("build all CMAKE_C_FLAGS is  " ${CMAKE_C_FLAGS} ", build version is " ${CMAKE_BUILD_TYPE})

add_subdirectory(xroc-framework)
add_subdirectory(fasterrcnnmethod)
add_subdirectory(cnnmethod)
add_subdirectory(facesnapfiltermethod)
add_subdirectory(GradingMethod)
add_subdirectory(SnapshotMethod)
add_subdirectory(xpluginflow)
add_subdirectory(xpluginflow_msgtype)
add_subdirectory(vioplugin)
add_subdirectory(hbipcplugin)
add_subdirectory(smartplugin)


