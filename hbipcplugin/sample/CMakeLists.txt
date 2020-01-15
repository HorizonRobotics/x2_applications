cmake_minimum_required(VERSION 2.8)
# set compiler option
option(CPU "build for CPU" OFF)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

link_directories(
        ${PROJECT_SOURCE_DIR}/lib
)

# 添加依赖
## base deps
if (PLATFORM_WIN)
    set(BASE_LIBRARIES gtest.lib hobotlog.lib)
else ()
    set(BASE_DEPS -ldl -lm -lpthread -lrt)
    #set(BASE_LIBRARIES ${BASE_LIBRARIES} hbipcplugin xpluginflowd libgtest.a libhobotlog.a libprotobuf.a libhbipc_cp.a libjsoncpp.a)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} ${BASE_DEPS})
endif ()

set(SOURCE_FILES
        sample.cpp
        )
# 添加依赖
add_executable(hbipc_sample ${SOURCE_FILES})
target_link_libraries(hbipc_sample ${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES})
