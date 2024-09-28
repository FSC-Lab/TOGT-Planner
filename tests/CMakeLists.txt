project(togt_tests)

include(FetchContent)

FetchContent_Declare(
  GTest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
  SYSTEM FIND_PACKAGE_ARGS NAMES GTest CONFIG 1.15.2)
FetchContent_MakeAvailable(GTest)
include(GoogleTest)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PUBLIC test_aos.cpp test_togt.cpp
                                      test_aos_togt.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE TOGT-Planner::Planner
                                              GTest::gtest GTest::gtest_main)
gtest_discover_tests(${PROJECT_NAME})
