cmake_minimum_required(VERSION 2.8)
# 项目名称
project(smartplugin)

set(ARCH_LIBS bpu_predict cnn_intf vio cam fb)
set(Method_LIBS FasterRCNNMethod MOTMethod
                ipc_tracking xroc-imagetools
                CNNMethod FaceSnapFilterMethod
                GradingMethod SnapShotMethod
                xroc-framework)
set(Method_LIBS_DEBUG FasterRCNNMethodd MOTMethod
                      ipc_tracking xroc-imagetools
                      CNNMethodd FaceSnapFilterMethodd
                      GradingMethodd SnapShotMethodd
                      xroc-frameworkd)

# 编译模式
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()


if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Ofast ")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC ")
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
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
set(BASE_LIBRARIES ${BASE_LIBRARIES} hobotlog  vision_type_util jsoncpp protobuf yuv turbojpeg ${ARCH_LIBS})
set(OpenCVLib opencv_world)
set(OpenCVDeps protobuf)
set(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})
set(XPLUGIN_LIBS hbipcplugin vioplugin xpluginflow_msgtype xpluginflow)
set(XPLUGIN_LIBS_DEBUG hbipcplugind vioplugind xpluginflow_msgtyped xpluginflowd)
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_link_libraries(${PROJECT_NAME} ${XPLUGIN_LIBS} ${Method_LIBS} ${BASE_LIBRARIES})
else ()
    target_link_libraries(${PROJECT_NAME} ${XPLUGIN_LIBS_DEBUG}  ${Method_LIBS} ${BASE_LIBRARIES})
endif ()

add_subdirectory(test)
add_subdirectory(sample)

# make install才会安装
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
install(TARGETS smartplugin
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
