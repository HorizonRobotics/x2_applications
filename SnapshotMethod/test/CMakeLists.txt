cmake_minimum_required(VERSION 2.8)

option(SHARED_LIB "build shared lib" OFF)

set(SOURCE_FILES
  method_factory.cpp
  src/test_support.cpp
)

set(OpenCVLib opencv_world)
# set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

include_directories(
        include
)

set(BASE_LIBRARIES xroc-framework
        ${OpenCVLib}
        ${OpenCVDeps}
        jsoncpp.a pthread dl)

if (${SHARED_LIB})
    add_library(snapshot_shared SHARED ${SOURCE_FILES})
    target_link_libraries(snapshot_shared ${PROJECT_NAME} -Wl,--whole-archive  -Wl,--no-whole-archive ${BASE_LIBRARIES})

    add_executable(snapshot_example snapshot_example.cpp)
    target_link_libraries(snapshot_example snapshot_shared)

    #add_executable(select_testing snap_testing.cpp)
    #target_link_libraries(select_testing select_shared)

    add_executable(gtest_snapshot gtest_Snapshot_Method.cpp)
    target_link_libraries(gtest_snapshot snapshot_shared pthread gtest)
else()
    add_executable(snapshot_example ${SOURCE_FILES} snapshot_example.cpp)
    target_link_libraries(snapshot_example ${PROJECT_NAME} -Wl,--whole-archive  -Wl,--no-whole-archive ${BASE_LIBRARIES})

    add_executable(gtest_snapshot ${SOURCE_FILES} gtest_Snapshot_Method.cpp)
    target_link_libraries(gtest_snapshot ${PROJECT_NAME} -Wl,--whole-archive  -Wl,--no-whole-archive ${BASE_LIBRARIES} pthread gtest)
endif()
