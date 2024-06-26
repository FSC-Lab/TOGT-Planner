# Setup catkin simple
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++17)

cs_add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -fno-finite-math-only
  -Wall
  # -Werror
  -Wpedantic
  -Wunused
  -Wno-unused-parameter
  -Wundef
  -Wcast-align
  -Wmissing-include-dirs
  # -Wnon-virtual-dtor
  -Wredundant-decls
  -Wodr
  -Wunreachable-code
  -Wno-unknown-pragmas
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wmissing-declarations
    # To keep the compiler calm about Eigen
    -Wno-sign-conversion
    -Wno-implicit-int-float-conversion
    -Wno-c99-extensions
    -Wno-implicit-int-conversion
  )
endif()

# Build tests
if(BUILD_TESTS)
  catkin_add_gtest(drolib_tests ${TEST_SOURCES})
  target_link_libraries(drolib_tests
    ${PROJECT_NAME}
    gtest
    gtest_main
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
    )
endif()

# Finish catkin simple
cs_install()
cs_export()
