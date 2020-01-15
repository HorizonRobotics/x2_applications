cmake_minimum_required(VERSION 2.8)

option(SHARED_LIB "build shared lib" OFF)

set(SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/method_factory.cpp
)

set(BASE_LIBRARIES jsoncpp.a pthread dl)

if (${SHARED_LIB})
    add_library(grading_shared SHARED ${SOURCE_FILES})
    target_link_libraries(grading_shared ${PROJECT_NAME} -Wl,--whole-archive xroc-framework -Wl,--no-whole-archive ${BASE_LIBRARIES})

    add_executable(grading_example grading_example.cpp)
    target_link_libraries(grading_example grading_shared)

    add_executable(gtest_grading gtest_Grading_Method.cpp)
    target_link_libraries(gtest_grading grading_shared pthread gtest)
else()
    add_executable(grading_example ${SOURCE_FILES} grading_example.cpp)
    target_link_libraries(grading_example ${PROJECT_NAME})

    add_executable(gtest_grading ${SOURCE_FILES} gtest_Grading_Method.cpp)
    target_link_libraries(gtest_grading ${PROJECT_NAME} pthread gtest)
endif()
