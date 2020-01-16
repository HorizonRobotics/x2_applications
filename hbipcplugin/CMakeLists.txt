cmake_minimum_required(VERSION 2.8)

# 项目名称
project(hbipcplugin)

# 编译参数
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
        ${PROJECT_SOURCE_DIR}/include/message
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
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
set(BASE_LIBRARIES ${BASE_LIBRARIES}  protobuf hobotlog vision_type_util jsoncpp)
set(XPLUGIN_LIBS xpluginflow_msgtype xpluginflow)
set(XPLUGIN_LIBS_DEBUG xpluginflow_msgtyped xpluginflowd)
set(BASE_LIBRARIES ${BASE_LIBRARIES})
set(LOCAL_LIBS hbipc_cp)
set(BASE_LIBRARIES ${BASE_LIBRARIES} ${LOCAL_LIBS})
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_link_libraries(${PROJECT_NAME} ${XPLUGIN_LIBS} ${BASE_LIBRARIES})
else ()
    target_link_libraries(${PROJECT_NAME} ${XPLUGIN_LIBS_DEBUG} ${BASE_LIBRARIES})
endif ()

#add_subdirectory(sample)
#add_subdirectory(test)

# make install才会安装
set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/deps)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
install(TARGETS hbipcplugin
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})

install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/config/
        DESTINATION ${MY_OUTPUT_ROOT}/config)
