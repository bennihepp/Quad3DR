set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

if("${EIGEN3_INCLUDE_DIR}" STREQUAL "")
  find_package(Eigen3 REQUIRED)
endif()
find_package(octomap REQUIRED)
#find_package(ompl REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

include_directories(
  include/
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
)

add_executable(quad_planner
  src/quad_planner_app.cpp
  src/quad_planner.cpp
  src/optimizing_rrt_planner.cpp
  src/rendering/visualizer.cpp
  src/rendering/octomap_renderer.cpp
  src/rendering/shader_program.cpp
  src/rendering/triangle_mesh.cpp
  src/rendering/lines.cpp
  src/rendering/utilities.cpp
  src/rendering/scene_object.cpp
)
target_link_libraries(quad_planner
  ${OMPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLFW_LIBRARIES}
  ${GLM_LIBRARIES}
  ${Boost_LIBRARIES}
)
