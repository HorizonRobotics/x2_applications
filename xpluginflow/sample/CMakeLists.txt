cmake_minimum_required(VERSION 2.8)
# set compiler option
option(CPU "build for CPU" OFF)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)
# test case的所有cpp文件
set(SOURCE_FILES
        sample_plugin.cpp
        )
# 添加依赖
## base deps
if (PLATFORM_WIN)
    set(BASE_LIBRARIES gtest.lib hobotlog.lib)
else ()
    set(BASE_DEPS -ldl -lm -lpthread -lrt)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} libgtest.a libhobotlog.a)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} ${BASE_DEPS})
endif ()

# hobot_util.cmake已经设置了在build/bin中生成可执行文件，默认应该在test目录里生成
add_executable(sample_plugin ${SOURCE_FILES})
target_link_libraries(sample_plugin ${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES})