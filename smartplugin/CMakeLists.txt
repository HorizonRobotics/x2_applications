cmake_minimum_required(VERSION 2.8)
# 项目名称
project(smartplugin)
#set(CMAKE_CXX_STANDARD 11)
option(RELEASE_LIB "build version of release" ON)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
add_definitions(-DHR_POSIX)
add_definitions(-DHR_LINUX)
add_definitions(-DX2)
set(ARCH_LIBS bpu_predict cnn_intf vio cam fb)
list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")
set(Method_LIBS FasterRCNNMethod MOTMethod ipc_tracking xroc-imagetools CNNMethod FaceSnapFilterMethod GradingMethod SnapShotMethod)

# 编译模式
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()

# 编译参数
if (NOT PLATFORM_WIN)
    if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Ofast ")
    else ()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC ")
    endif ()
endif ()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

# 添加头文件路径
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/include
        ${PROJECT_SOURCE_DIR}/deps/vision_type/include
        ${PROJECT_SOURCE_DIR}/deps/xroc-framework/include
        ${PROJECT_SOURCE_DIR}/deps/opencv/include
        ${PROJECT_SOURCE_DIR}/deps/MOTMethod/include
        ${PROJECT_SOURCE_DIR}/deps/FasterRCNNMethod/include
        ${PROJECT_SOURCE_DIR}/deps/protobuf/include
        ${PROJECT_SOURCE_DIR}/deps/vioplugin/include
        ${PROJECT_SOURCE_DIR}/deps/hbipcplugin/include
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow/include
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow_msgtype/include
        ${PROJECT_SOURCE_DIR}/deps/x2_prebuilt/include
        ${PROJECT_SOURCE_DIR}/deps/jsoncpp/include
        ${PROJECT_SOURCE_DIR}/deps/ipc_tracking/include
        ${PROJECT_SOURCE_DIR}/deps/bpu_predict/include
        ${PROJECT_SOURCE_DIR}/deps/gtest/include
        ${PROJECT_SOURCE_DIR}/deps/hbipc/include
        ${PROJECT_SOURCE_DIR}/deps/FaceSnapFilterMethod/include
        ${PROJECT_SOURCE_DIR}/deps/GradingMethod/include
        ${PROJECT_SOURCE_DIR}/deps/SnapShotMethod/include
        ${PROJECT_SOURCE_DIR}/deps/CNNMethod/include
        ${PROJECT_SOURCE_DIR}/deps/libjpeg-turbo/include
        ${PROJECT_SOURCE_DIR}/deps/libyuv/include
        ${PROJECT_SOURCE_DIR}/deps/xroc-imagetools/include
    )
link_directories(
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/lib
        ${PROJECT_SOURCE_DIR}/deps/vision_type/lib
        ${PROJECT_SOURCE_DIR}/deps/xroc-framework/lib
        ${PROJECT_SOURCE_DIR}/deps/opencv/lib
        ${PROJECT_SOURCE_DIR}/deps/MOTMethod/lib
        ${PROJECT_SOURCE_DIR}/deps/FasterRCNNMethod/lib
        ${PROJECT_SOURCE_DIR}/deps/protobuf/lib
        ${PROJECT_SOURCE_DIR}/deps/vioplugin/lib
        ${PROJECT_SOURCE_DIR}/deps/hbipcplugin/lib
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow/lib
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow_msgtype/lib
        ${PROJECT_SOURCE_DIR}/deps/x2_prebuilt/lib
        ${PROJECT_SOURCE_DIR}/deps/jsoncpp/lib
        ${PROJECT_SOURCE_DIR}/deps/ipc_tracking/lib
        ${PROJECT_SOURCE_DIR}/deps/bpu_predict/lib
        ${PROJECT_SOURCE_DIR}/deps/gtest/lib
        ${PROJECT_SOURCE_DIR}/deps/hbipc/lib
        ${PROJECT_SOURCE_DIR}/deps/FaceSnapFilterMethod/lib
        ${PROJECT_SOURCE_DIR}/deps/GradingMethod/lib
        ${PROJECT_SOURCE_DIR}/deps/SnapShotMethod/lib
        ${PROJECT_SOURCE_DIR}/deps/CNNMethod/lib
        ${PROJECT_SOURCE_DIR}/deps/libjpeg-turbo/lib
        ${PROJECT_SOURCE_DIR}/deps/libyuv/lib
        ${PROJECT_SOURCE_DIR}/deps/xroc-imagetools/lib     
)
# 头文件路径信息
file(GLOB PROJECT_INC
        "include/*.h"
        "include/*.hpp"
        "include/smartplugin/*.h"
        "include/smartplugin/utils/*.h"
        "include/horizon/vision/*.h"
        )

# 源文件路径信息
file(GLOB PROJECT_SRC
        "src/smartplugin/*.cpp"
        "src/utils/*.cpp"
        "src/smartplugin/xroc/*.cpp"
        )

set(SOURCE_FILES
        ${PROJECT_SRC}
        ${PROJECT_INC}
        )
# add_library的时候不需要target_link_library
add_library(${CMAKE_PROJECT_NAME} SHARED ${SOURCE_FILES})
set(BASE_LIBRARIES ${BASE_LIBRARIES} hobotlog xroc-framework vision_type_util jsoncpp protobuf yuv turbojpeg ${ARCH_LIBS})
set(OpenCVLib opencv_world)
set(OpenCVDeps protobuf)
set(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})
#target_link_libraries(${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES})

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_link_libraries(${CMAKE_PROJECT_NAME} xpluginflow_msgtype xpluginflow vioplugin ${Method_LIBS} ${BASE_LIBRARIES})
else ()
    target_link_libraries(${CMAKE_PROJECT_NAME} xpluginflow_msgtyped xpluginflowd vioplugind  ${Method_LIBS} ${BASE_LIBRARIES})
endif ()

add_subdirectory(test)
add_subdirectory(sample)

# make install才会安装
set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/output)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${CMAKE_PROJECT_NAME})
install(TARGETS smartplugin
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${CMAKE_PROJECT_NAME})
