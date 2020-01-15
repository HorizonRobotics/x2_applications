include_directories(
        include
)


set(SOURCE_FILES
        gtest_main.cc
        )

set(COMMON_DEPS
    zlib -lpthread -ldl
    )

add_executable(CNNMethod_unit_test ${SOURCE_FILES})

if (${PLATFORM_WIN})
     set(CMAKE_CXX_FLAGS_RELEASE "/MT")
     set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
     target_link_libraries(CNNMethod_unit_test debug gtestd)
     target_link_libraries(CNNMethod_unit_test optimized gtest)
else()
     target_link_libraries(CNNMethod_unit_test
                           ${COMMON_DEPS}
                           gtest)
endif()
