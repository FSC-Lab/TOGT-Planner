cmake_minimum_required(VERSION 3.0.2)
project(drolib)

message(STATUS "==================   TOGT Planner   ===================")

################################################################################
#Options
################################################################################

option(BUILD_TESTS "Building the tests" ON)
option(ENABLE_FAST "Build with optimizations for speed" ON)
option(UNSAFE_MATH "Build with -funsafe-math-optimizations: high impact on speed, but lowers floating point precision." ON)
option(ENABLE_PROFILING "Build for profiling" OFF)
option(ENABLE_PARALLEL "Build using openmp parallelization" OFF)
option(EIGEN_FROM_SYSTEM "Use the system-provided Eigen" ON)
option(DEBUG_LOGGING "Enable detailed logging" OFF)
set(
        EIGEN_ALTERNATIVE "" CACHE STRING
        "Path to alternative Eigen, autodownload if blank"
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# message(STATUS "BUILD_SHARED_LIBS is " ${BUILD_SHARED_LIBS})


################################################################################
#Finding Dependencies
################################################################################
message(STATUS "===============   Setup Dependencies   ================")

if (EIGEN_FROM_SYSTEM)
  find_package(Eigen3 3.3.4 QUIET)
  if (EIGEN3_FOUND)
    message(STATUS "Using system provided Eigen.")
  else ()
    message(STATUS "No sufficient Eigen version (3.3.4) found.")
    message(STATUS "Restoring to download Eigen sources.")
    include(cmake/eigen.cmake)
  endif ()
elseif (EIGEN_ALTERNATIVE STREQUAL "")
  include(cmake/eigen.cmake)
else ()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_ALTERNATIVE})
endif ()

#Check for ccache
if (NOT DEFINED CATKIN_DEVEL_PREFIX)
    find_program(CCACHE_PROGRAM ccache)
    if (CCACHE_PROGRAM)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    else ()
        message(STATUS "Build time could be improved with ccache!")
        message(STATUS "    sudo apt install ccache")
    endif ()
endif ()

################################################################################
#Setup Compilation
################################################################################
message(STATUS "===============   Setup Compilation   =================")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=1048576)
if (ENABLE_PARALLEL)
    find_package(OpenMP REQUIRED)
    message(STATUS "Enabling parallelization!")
    set(CMAKE_CXX_PAR_FLAGS " -fopenmp")
else ()
  add_definitions(-DEIGEN_DONT_PARALLELIZE)
endif ()

include_directories(include)
include_directories(AFTER tests)
include_directories(BEFORE SYSTEM ${EIGEN3_INCLUDE_DIRS})

#Set default build type
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif ()
endif ()

#Fix build type for profiling
if (ENABLE_PROFILING)
  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Switching build type to RelWithDebInfo for profiling")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
  endif()
  add_definitions(-fno-omit-frame-pointer)
endif()

#Add c++ flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g") # Insert debug information

#Architectural flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "arm64")
    message(STATUS "Using ARM64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    message(STATUS "CATKIN_DEVEL_PREFIX is " ${CATKIN_DEVEL_PREFIX})
    if (NOT DEFINED CATKIN_DEVEL_PREFIX)
        set(CMAKE_CXX_ARCH_FLAGS " -march=native")
    endif ()
endif ()

#Optimized flags
if (ENABLE_FAST)
    message(STATUS "Enabling fast optimization flags!")
    set(CMAKE_CXX_FAST_FLAGS " -O3")
else ()
    set(CMAKE_CXX_FAST_FLAGS " -O0") # No optimization
endif ()
if (UNSAFE_MATH)
    message(STATUS "Using -funsafe-math-optimizations")
    set(CMAKE_CXX_FAST_FLAGS "${CMAKE_CXX_FAST_FLAGS} -funsafe-math-optimizations")
endif ()

#DebugLogging
if(DEBUG_LOGGING)
  message(STATUS "Enable Debug Logs!")
  add_definitions(-DDEBUG_LOG)
endif()

#Summarize Flags
set(CMAKE_CXX_FLAGS_RELEASE
  "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FAST_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} ${CMAKE_CXX_PAR_FLAGS} ${CMAKE_CXX_BLAS_FLAGS}")
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "The activated CXX RELEASE configuration is:\n ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "The activated CXX DEBUG configuration is:\n ${CMAKE_CXX_FLAGS_DEBUG}")


################################################################################
#Specify Build Resources
################################################################################
message(STATUS "==================   Setup Build   ====================")

file(GLOB_RECURSE SOURCES
        src/*.cpp
        )

#Get all test-related source files
file(GLOB_RECURSE TEST_SOURCES
        tests/*.cpp
        )

################################################################################
# Optional Catkin Build
################################################################################

if (DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "============   Building with -- catkin --  ============")
    include(cmake/catkin.cmake)
    return()
endif ()

################################################################################
# Setup Build
################################################################################

# Setup Testing and Benchmark
if (BUILD_TESTS)
    include(cmake/gtest.cmake)
    enable_testing()
endif ()

message(STATUS "SOURCES is " ${SOURCES})
message(STATUS "TEST_SOURCES is " ${TEST_SOURCES})

if (NOT SOURCES)
    set(LIBRARY_NAME)
else ()
    add_library(${PROJECT_NAME} ${SOURCES})
    target_link_libraries(${PROJECT_NAME}
      PUBLIC
      $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>)
    target_compile_options(${PROJECT_NAME} PRIVATE
      -fno-finite-math-only
      -Wall
      -Wextra
      -Wpedantic
      -Werror
      -Wunused
      -Wno-unused-parameter
      -Wundef
      -Wcast-align
      -Wmissing-declarations
      -Wmissing-include-dirs
      -Wnon-virtual-dtor
      -Wredundant-decls
      -Wodr
      -Wunreachable-code
      -Wno-unknown-pragmas
    )
    # To keep the compiler calm about Eigen
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      target_compile_options(${PROJECT_NAME} PRIVATE
        -Wno-sign-conversion
        -Wno-implicit-int-float-conversion
        -Wno-c99-extensions
        -Wno-implicit-int-conversion
      )
    endif()
    set(LIBRARY_NAME ${PROJECT_NAME})
endif ()


# Build tests
if (BUILD_TESTS)
    add_executable(tests ${TEST_SOURCES})
    target_compile_options(tests PRIVATE
      -fno-finite-math-only
      -Wall                   # Show all warnings
      -Wextra                 # Show extra error information
      -Wpedantic              # Issue all the warnings demanded by strict ISO C and ISO C++
      -Werror                 # Raise all warnings to errors
      -Wunused                # Warn whatever is assigned to, but unused
      -Wno-unused-parameter
      -Wundef                 # Warn if an undefined identifier is evaluated in an #if directive. 
      -Wcast-align            # Warn whenever a pointer is cast such that the required alignment of the target is increased
      -Wmissing-declarations  # Warn if a global function is defined without a previous declaration
      -Wmissing-include-dirs  # Warn if a user-supplied include directory does not exist.
      -Wnon-virtual-dtor      
      -Wredundant-decls       # Warn if anything is declared more than once in the same scope
      -Wodr
      -Wunreachable-code
      -Wno-unknown-pragmas
    )
    target_link_libraries(tests
            ${LIBRARY_NAME}
            gtest
            gtest_main
            $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
            )
    add_test(tests tests)
endif ()

message(STATUS "===========   The TOGT Planner has been built successfully!   ==============")


