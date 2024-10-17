cmake_minimum_required(VERSION 3.25.0)
project(drolib)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3.4 REQUIRED)

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=1048576)

add_library(${PROJECT_NAME})
target_sources(
  ${PROJECT_NAME}
  PUBLIC FILE_SET
         "public_headers"
         TYPE
         "HEADERS"
         BASE_DIRS
         "include"
         FILES
         include/drolib/base/corridor_base.hpp
         include/drolib/base/parameter_base.hpp
         include/drolib/base/shape_base.hpp
         include/drolib/corridor/free_corridor.hpp
         include/drolib/corridor/prisma_corridor.hpp
         include/drolib/corridor/single_ball.hpp
         include/drolib/file/file_utils.hpp
         include/drolib/file/filesystem.hpp
         include/drolib/file/yaml.hpp
         include/drolib/geometry/geoutils.hpp
         include/drolib/geometry/quickhull.hpp
         include/drolib/math/fast_atan2.hpp
         include/drolib/math/gravity.hpp
         include/drolib/math/median.hpp
         include/drolib/math/min_max_recorder.hpp
         include/drolib/planner/angle.hpp
         include/drolib/planner/traj_data.hpp
         include/drolib/planner/traj_params.hpp
         include/drolib/planner/traj_solver.hpp
         include/drolib/planner/waypoint.hpp
         include/drolib/polynomial/piecewise_polynomial.hpp
         include/drolib/polynomial/poly_traj_utils.hpp
         include/drolib/polynomial/polynomial.hpp
         include/drolib/polynomial/trajectory.hpp
         include/drolib/race/race_params.hpp
         include/drolib/race/race_planner.hpp
         include/drolib/race/race_track.hpp
         include/drolib/solver/banded_system.hpp
         include/drolib/solver/lbfgs.hpp
         include/drolib/solver/lbfgs_params.hpp
         include/drolib/solver/minco_snap.hpp
         include/drolib/solver/root_finder.hpp
         include/drolib/solver/sdlp.hpp
         include/drolib/system/minco_snap_trajectory.hpp
         include/drolib/system/quadrotor_manifold.hpp
         include/drolib/system/quadrotor_params.hpp
         include/drolib/type/command.hpp
         include/drolib/type/imu_sample.hpp
         include/drolib/type/odom.hpp
         include/drolib/type/quad_state.hpp
         include/drolib/type/set_point.hpp
         include/drolib/type/types.hpp
         include/drolib/utils/logger.hpp
         include/drolib/utils/timer.hpp
  PRIVATE src/base/parameter_base.cpp
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
          src/shape/ball.cpp
          src/shape/free_point.cpp
          src/shape/hexagon.cpp
          src/shape/pentagon.cpp
          src/shape/polygon.cpp
          src/shape/polyhedron.cpp
          src/shape/rectangle.cpp
          src/shape/triangle.cpp
          src/solver/lbfgs_params.cpp
          src/system/minco_snap_trajectory.cpp
          src/system/quadrotor_manifold.cpp
          src/system/quadrotor_params.cpp
          src/type/command.cpp
          src/type/quad_state.cpp
          src/utils/logger.cpp
          src/utils/timer.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)

endif(BUILD_TESTING)
