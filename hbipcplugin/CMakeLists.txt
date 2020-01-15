cmake_minimum_required(VERSION 2.8)

# 项目名称
project(hbipcplugin)
set(CMAKE_CXX_STANDARD 11)
option(RELEASE_LIB "build version of release" ON)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
add_definitions(-DHR_POSIX)
add_definitions(-DHR_LINUX)
add_definitions(-DX2)
list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")

# 编译参数
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Ofast ")

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
# 添加头文件路径
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/message
        ${PROJECT_SOURCE_DIR}/deps/jsoncpp/include
        ${PROJECT_SOURCE_DIR}/deps/protobuf/include
        ${PROJECT_SOURCE_DIR}/deps/hbipc/include
        ${PROJECT_SOURCE_DIR}/deps/x2_prebuilt/include
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/include
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow/include
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow_msgtype/include
        ${PROJECT_SOURCE_DIR}/deps/vision_type/include
)
link_directories(
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/deps/jsoncpp/lib
        ${PROJECT_SOURCE_DIR}/deps/protobuf/lib
        ${PROJECT_SOURCE_DIR}/deps/hbipc/lib
        ${PROJECT_SOURCE_DIR}/deps/x2_prebuilt/lib
        ${PROJECT_SOURCE_DIR}/deps/hobotlog/lib
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow/lib
        ${PROJECT_SOURCE_DIR}/deps/xpluginflow_msgtype/lib
        ${PROJECT_SOURCE_DIR}/deps/vision_type/lib
)
# 头文件路径信息
file(GLOB PROJECT_INC
        "include/*.h"
        "include/hbipcplugin/*.h"
        "include/utils/*.h"
        "include/message/*.h"
        )

# 源文件路径信息
file(GLOB PROJECT_SRC
        "src/hbipcplugin/*.cpp"
        "src/message/*.cc"
        "src/utils/*.cpp"
        )

set(SOURCE_FILES
        ${PROJECT_SRC}
        ${PROJECT_INC}
        )
# add_library的时候不需要target_link_library
add_library(${CMAKE_PROJECT_NAME} SHARED ${SOURCE_FILES})
set(BASE_LIBRARIES ${BASE_LIBRARIES} xpluginflow_msgtype protobuf hobotlog vision_type_util jsoncpp)
set(XPLUGIN_LIBS xpluginflow)
set(BASE_LIBRARIES ${XPLUGIN_LIBS} ${BASE_LIBRARIES})
set(LOCAL_LIBS hbipc_cp)
set(BASE_LIBRARIES ${BASE_LIBRARIES} ${LOCAL_LIBS})
#set(OpenCVLib opencv_world)
#set(OpenCVDeps libjasper libjpeg-turbo libpng libprotobuf libtiff libwebp quirc tegra_hal zlib)
#SET(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})
target_link_libraries(${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES})

#add_subdirectory(sample)
#add_subdirectory(test)

# make install才会安装
set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/deps)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${CMAKE_PROJECT_NAME})
install(TARGETS hbipcplugin
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${CMAKE_PROJECT_NAME})

install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/config/
        DESTINATION ${MY_OUTPUT_ROOT}/config)
