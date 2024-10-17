project(tests)
include(FetchContent)
FetchContent_Declare(
  GTest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2)

FetchContent_MakeAvailable(GTest)
include(GoogleTest)
add_executable(tests)
target_sources(tests PRIVATE test_aos.cpp test_aos_togt.cpp test_togt.cpp)
target_compile_definitions(tests PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
target_link_libraries(tests PRIVATE GTest::gtest GTest::gtest_main
                                    ${CMAKE_PROJECT_NAME})
gtest_discover_tests(tests)
