cmake_minimum_required(VERSION 2.8)

project(GradingMethod)
message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}" )
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC")
endif()

include_directories(
        include
)

set(BASE_LIBRARIES xroc-framework hobotlog jsoncpp.a rt dl pthread)

set(SOURCE_FILES
        src/method/GradingMethod.cpp
        src/method/WeightGrading.cpp
)

set(GRADING_LIBRARIES rt)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${GRADING_LIBRARIES}  ${BASE_LIBRARIES} )

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/GradingMethod.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

add_subdirectory(test)
