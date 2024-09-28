cmake_minimum_required(VERSION 3.24.0)
project(TOGT-Planner)

include(FetchContent)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG v3.4.0
  SYSTEM FIND_PACKAGE_ARGS NAMES Eigen3 CONFIG 3.4.0)
FetchContent_MakeAvailable(Eigen3)
add_library(TOGT-Planner_Shapes)
target_sources(
  TOGT-Planner_Shapes
  PUBLIC src/shape/ball.cpp
         src/shape/free_point.cpp
         src/shape/hexagon.cpp
         src/shape/pentagon.cpp
         src/shape/polygon.cpp
         src/shape/polyhedron.cpp
         src/shape/rectangle.cpp
         src/shape/triangle.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         ${CMAKE_CURRENT_SOURCE_DIR}/include
         FILES
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/ball.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/free_point.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/hexagon.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/pentagon.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/polygon.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/polyhedron.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/rectangle.h
         ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/shape/triangle.h)
target_compile_features(TOGT-Planner_Shapes PUBLIC cxx_std_17)
target_link_libraries(TOGT-Planner_Shapes PUBLIC Eigen3::Eigen)
set_target_properties(TOGT-Planner_Shapes PROPERTIES EXPORT_NAME Shapes)
add_library(TOGT-Planner::Shapes ALIAS TOGT-Planner_Shapes)

add_library(TOGT-Planner_Planner)
target_sources(
  TOGT-Planner_Planner
  PUBLIC src/base/parameter_base.cpp
         src/base/shape_base.cpp
         src/corridor/free_corridor.cpp
         src/corridor/prisma_corridor.cpp
         src/corridor/single_ball.cpp
         src/math/math_utils.cpp
         src/planner/traj_data.cpp
         src/planner/traj_params.cpp
         src/planner/traj_solver.cpp
         src/polynomial/cubic_spline.cpp
         src/polynomial/cubic_spline3d.cpp
         src/race/race_params.cpp
         src/race/race_planner.cpp
         src/race/race_track.cpp
         src/rotation/rotation_utils.cpp
         src/solver/lbfgs_params.cpp
         src/system/minco_snap_trajectory.cpp
         src/system/quadrotor_manifold.cpp
         src/system/quadrotor_params.cpp
         src/type/command.cpp
         src/type/quad_state.cpp
         src/utils/logger.cpp
         src/utils/timer.cpp
  PRIVATE
    FILE_SET
    HEADERS
    BASE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/base/corridor_base.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/base/parameter_base.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/base/shape_base.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/corridor/free_corridor.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/corridor/prisma_corridor.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/corridor/single_ball.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/file/file_utils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/file/filesystem.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/file/yaml.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/geometry/geoutils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/geometry/quickhull.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/math/fast_atan2.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/math/gravity.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/math/median.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/math/min_max_recorder.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/planner/angle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/planner/traj_data.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/planner/traj_params.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/planner/traj_solver.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/planner/waypoint.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/polynomial/piecewise_polynomial.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/polynomial/poly_traj_utils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/polynomial/polynomial.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/polynomial/trajectory.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/race/race_params.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/race/race_planner.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/race/race_track.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/solver/banded_system.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/solver/lbfgs.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/solver/lbfgs_params.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/solver/minco_snap.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/solver/root_finder.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/solver/sdlp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/system/minco_snap_trajectory.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/system/quadrotor_manifold.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/system/quadrotor_params.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/type/command.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/type/imu_sample.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/type/odom.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/type/quad_state.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/type/set_point.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/type/types.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/utils/logger.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drolib/utils/timer.hpp)
target_compile_features(TOGT-Planner_Planner PUBLIC cxx_std_17)
target_link_libraries(TOGT-Planner_Planner PUBLIC Eigen3::Eigen
                                                  TOGT-Planner::Shapes)

set_target_properties(TOGT-Planner_Planner PROPERTIES EXPORT_NAME Planner)
add_library(TOGT-Planner::Planner ALIAS TOGT-Planner_Planner)

# Build tests
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
