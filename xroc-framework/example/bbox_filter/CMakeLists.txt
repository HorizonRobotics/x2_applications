cmake_minimum_required(VERSION 2.8)
include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/include
)
set(BASE_LIBRARIES hobotlog jsoncpp pthread dl)

set(SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/method/bbox_filter.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/method_factory.cpp
)
file(GLOB_RECURSE SOURCE_SRC "src/data_types/*.cpp" "src/data_types/*.cc")

add_library(bbox_filter SHARED ${SOURCE_FILES} ${SOURCE_SRC})
target_link_libraries(bbox_filter -Wl,--whole-archive xroc-framework -Wl,--no-whole-archive ${BASE_LIBRARIES})

add_executable(bbox_filter_example src/bbox_filter_main.cpp)
target_link_libraries(bbox_filter_example bbox_filter ${BASE_LIBRARIES})

add_executable(c_bbox_filter_example src/c_bbox_filter_main.c)
target_link_libraries(c_bbox_filter_example bbox_filter ${BASE_LIBRARIES})

add_executable(profiler_test src/profiler_test.cpp)
target_link_libraries(profiler_test bbox_filter ${BASE_LIBRARIES})

add_executable(disable_method_test src/disable_method_test.cpp)
target_link_libraries(disable_method_test bbox_filter ${BASE_LIBRARIES})