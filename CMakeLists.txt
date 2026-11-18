cmake_minimum_required(VERSION 3.16)
project(GuidedQuad VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_library(guidedquad_core src/guide_graph.cpp src/patch_solver.cpp src/surface_projection.cpp src/quad_intersections.cpp)
target_include_directories(guidedquad_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
  target_compile_options(guidedquad_core PRIVATE /W4)
else()
  target_compile_options(guidedquad_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_executable(guidedquad_cli src/cli.cpp)
target_link_libraries(guidedquad_cli PRIVATE guidedquad_core)
include(CTest)
if(BUILD_TESTING)
  add_executable(guidedquad_tests tests/test_guides.cpp)
  target_link_libraries(guidedquad_tests PRIVATE guidedquad_core)
  add_test(NAME guidedquad_guides COMMAND guidedquad_tests)
  add_executable(guidedquad_patch_tests tests/test_patch_solver.cpp)
  target_link_libraries(guidedquad_patch_tests PRIVATE guidedquad_core)
  add_test(NAME guidedquad_patch COMMAND guidedquad_patch_tests)
  add_executable(guidedquad_projection_tests tests/test_surface_projection.cpp)
  target_link_libraries(guidedquad_projection_tests PRIVATE guidedquad_core)
  add_test(NAME guidedquad_projection COMMAND guidedquad_projection_tests)
  add_executable(guidedquad_collision_tests tests/test_quad_intersections.cpp)
  target_link_libraries(guidedquad_collision_tests PRIVATE guidedquad_core)
  add_test(NAME guidedquad_collision COMMAND guidedquad_collision_tests)
  add_test(NAME guidedquad_collision_cli_clean COMMAND guidedquad_cli
    check-quads ${CMAKE_CURRENT_SOURCE_DIR}/tests/separated_quads.obj)
  add_test(NAME guidedquad_collision_cli_crossing COMMAND guidedquad_cli
    check-quads ${CMAKE_CURRENT_SOURCE_DIR}/tests/colliding_quads.obj)
  set_tests_properties(guidedquad_collision_cli_crossing PROPERTIES WILL_FAIL TRUE)
  add_test(NAME guidedquad_patch_cli COMMAND guidedquad_cli
    patch-loop ${CMAKE_CURRENT_SOURCE_DIR}/tests/square.obj
    ${CMAKE_CURRENT_BINARY_DIR}/guidedquad_square_quads.obj 0 1 2 3 5 5)
  add_test(NAME guidedquad_projection_cli COMMAND guidedquad_cli
    project-loop ${CMAKE_CURRENT_SOURCE_DIR}/tests/square.obj
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/curved_surface.obj
    ${CMAKE_CURRENT_BINARY_DIR}/guidedquad_curved_projected.obj 0 1 2 3 8 8 2 1)
  add_test(NAME guidedquad_projection_oriented_cli COMMAND guidedquad_cli
    project-loop ${CMAKE_CURRENT_SOURCE_DIR}/tests/square.obj
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/curved_surface.obj
    ${CMAKE_CURRENT_BINARY_DIR}/guidedquad_oriented_projected.obj 0 1 2 3 8 8 2 1 0.5)
endif()
