option(WITH_OPENGL_OFFSCREEN "Offscreen OpenGL support" On)
option(WITH_PROFILING "Profiling support" Off)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

if("${EIGEN3_INCLUDE_DIR}" STREQUAL "")
    find_package(Eigen3 REQUIRED)
endif()
find_package(octomap REQUIRED)
find_package(octovis REQUIRED)
find_package(ompl REQUIRED)

#find_package(GLEW REQUIRED)
#find_package(glm REQUIRED)
#find_package(PkgConfig REQUIRED)
#pkg_search_module(GLFW REQUIRED glfw3)

# CUDA
find_package(CUDA REQUIRED)
# Set nvcc flags
list(APPEND CUDA_NVCC_FLAGS --compiler-options -fno-strict-aliasing -use_fast_math)
list(APPEND CUDA_NVCC_FLAGS -lineinfo)
list(APPEND CUDA_NVCC_FLAGS -Xptxas -dlcm=cg)
#list(APPEND CUDA_NVCC_FLAGS --debug)
#list(APPEND CUDA_NVCC_FLAGS --device-debug)
if(CUDA_VERSION_MAJOR LESS 7)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_11,code=sm_11)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_12,code=sm_12)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_13,code=sm_13)
elseif(CUDA_VERSION_MAJOR EQUAL 7)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_30,code=sm_30)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_35,code=sm_35)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_50,code=sm_50)
elseif(CUDA_VERSION_MAJOR GREATER 7)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_61,code=sm_61)
endif()
add_definitions(-DWITH_CUDA=1)

# SQLITE3
find_package(SQLite3)

if(WITH_OPENGL_OFFSCREEN)
    # Whether to use OpenGL in the viewpoint planner (used to compute voxel normals based on poisson mesh)
    add_definitions(-DWITH_OPENGL_OFFSCREEN=1)
endif()

# Enable debugger breaking on BH_ASSERT macros
add_definitions(-DAIT_ASSERT_BREAK=1)
add_definitions(-DBH_ASSERT_BREAK=1)

# Qt settings
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to automatically include resources
set(CMAKE_AUTORCC ON)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenGL REQUIRED)
#find_package(Qt4 REQUIRED QtCore QtGui QtXml QtOpenGL)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5WebSockets REQUIRED)

# Qt4 was built with -reduce-relocations.
if(Qt5_POSITION_INDEPENDENT_CODE)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(WITH_PROFILING)
    add_definitions(-DBH_WITH_PROFILING=1)
    set(GPERFTOOLS_LIBRARIES "profiler")
endif()

include_directories(
    ../external/QGLViewer
    ${OCTOMAP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${SQLITE3_INCLUDE_DIRS}
)

add_executable(occupancy_map_from_sens
    # Executable
    src/exe/occupancy_map_from_sens.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
    # Octree
    src/octree/occupancy_map.h
    src/octree/occupancy_map.hxx
    src/octree/occupancy_map_tree_navigator.hxx
    src/octree/occupancy_map.cpp
    src/octree/occupancy_node.h
    src/octree/occupancy_node.cpp
)
target_link_libraries(occupancy_map_from_sens
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(occupancy_map_from_colmap
    # Executable
    src/exe/occupancy_map_from_colmap.cpp
    # Octree
    src/octree/occupancy_map.h
    src/octree/occupancy_map.hxx
    src/octree/occupancy_map_tree_navigator.hxx
    src/octree/occupancy_map.cpp
    src/octree/occupancy_node.h
    src/octree/occupancy_node.cpp
    # Rendering
    src/reconstruction/sparse_reconstruction.h
    src/reconstruction/sparse_reconstruction.cpp
    src/reconstruction/dense_reconstruction.h
    src/reconstruction/dense_reconstruction.cpp
)
target_link_libraries(occupancy_map_from_colmap
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(occupancy_map_from_mesh
    # Executable
    src/exe/occupancy_map_from_mesh.cpp
    # Octree
    src/octree/occupancy_map.h
    src/octree/occupancy_map.hxx
    src/octree/occupancy_map_tree_navigator.hxx
    src/octree/occupancy_map.cpp
    src/octree/occupancy_node.h
    src/octree/occupancy_node.cpp
    # Rendering
    src/reconstruction/sparse_reconstruction.h
    src/reconstruction/sparse_reconstruction.cpp
    src/reconstruction/dense_reconstruction.h
    src/reconstruction/dense_reconstruction.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(occupancy_map_from_mesh
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(transform_mesh WIN32
    # Executable
    src/exe/transform_mesh.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(transform_mesh
    ${Boost_LIBRARIES}
)

add_executable(clip_mesh WIN32
    # Executable
    src/exe/clip_mesh.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(clip_mesh
    viewpoint_planner_common_objects
)
target_link_libraries(clip_mesh
    ${Boost_LIBRARIES}
)

add_executable(fuse_point_cloud WIN32
    # Executable
    src/exe/fuse_point_cloud.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(fuse_point_cloud
    ${Boost_LIBRARIES}
)

add_executable(evaluate_dense_points WIN32
    # Executable
    src/exe/evaluate_dense_points.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(evaluate_dense_points
    ${Boost_LIBRARIES}
)

add_executable(create_colmap_mvs_files WIN32
    # Executable
    src/exe/create_colmap_mvs_files.cpp
    # BH
    ../src/bh/utilities.cpp
)
target_link_libraries(create_colmap_mvs_files
    viewpoint_planner_common_objects
)
target_link_libraries(create_colmap_mvs_files
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(merge_viewpoint_paths WIN32
    # Executable
    src/exe/merge_viewpoint_paths.cpp
    # BH
    ../src/bh/utilities.cpp
)
target_link_libraries(merge_viewpoint_paths
    viewpoint_planner_common_objects
)
target_link_libraries(merge_viewpoint_paths
    ${Boost_LIBRARIES}
)

add_executable(image_matcher WIN32
    # Executable
    src/exe/image_matcher.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
    # Offscreen rendering resources
    src/shaders_offscreen.qrc
)
if(WITH_OPENGL_OFFSCREEN)
    target_link_libraries(image_matcher
        ${OPENGL_LIBRARIES}
    )
    target_link_libraries(image_matcher Qt5::Core Qt5::Gui Qt5::OpenGL)
endif()
target_link_libraries(image_matcher
    viewpoint_planner_common_objects
)
target_link_libraries(image_matcher
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${SQLITE3_LIBRARIES}
)

add_executable(image_matcher_test WIN32
    # Executable
    src/exe/image_matcher_test.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
    # Offscreen rendering resources
    src/shaders_offscreen.qrc
)
if(WITH_OPENGL_OFFSCREEN)
    target_link_libraries(image_matcher_test
        ${OPENGL_LIBRARIES}
    )
    target_link_libraries(image_matcher_test Qt5::Core Qt5::Gui Qt5::OpenGL)
endif()
target_link_libraries(image_matcher_test
    viewpoint_planner_common_objects
)
target_link_libraries(image_matcher_test
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${SQLITE3_LIBRARIES}
)

add_executable(image_match_invalidator WIN32
    # Executable
    src/exe/image_match_invalidator.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(image_match_invalidator
    viewpoint_planner_common_objects
)
target_link_libraries(image_match_invalidator
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${SQLITE3_LIBRARIES}
)

set(VIEWPOINT_PLANNER_SOURCES_COMMON
    # BH
    ../src/bh/utilities.cpp
    ../src/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
    # BVH
    src/bvh/bvh.h
    src/bvh/bvh.cu
    src/bvh/bvh.cuh
    # Graph
    #src/graph/graph.h
    #src/graph/graph_boost.h
    # Octree
    src/octree/occupancy_map.h
    src/octree/occupancy_map.hxx
    src/octree/occupancy_map_tree_navigator.hxx
    src/octree/occupancy_map.cpp
    src/octree/occupancy_node.h
    src/octree/occupancy_node.cpp
    # Planner
    src/planner/occupied_tree.h
    src/planner/viewpoint.h
    src/planner/viewpoint.cpp
    src/planner/viewpoint_raycast.h
    src/planner/viewpoint_raycast.cpp
    src/planner/viewpoint_score.h
    src/planner/viewpoint_score.cpp
    src/planner/viewpoint_offscreen_renderer.h
    src/planner/viewpoint_offscreen_renderer.cpp
    src/planner/viewpoint_planner.h
    src/planner/viewpoint_planner.cpp
    src/planner/viewpoint_planner.hxx
    src/planner/viewpoint_planner_raycast.cpp
    src/planner/viewpoint_planner_sampling.cpp
    src/planner/viewpoint_planner_motion.cpp
    src/planner/viewpoint_planner_scoring.cpp
    src/planner/viewpoint_planner_export.cpp
    src/planner/viewpoint_planner_serialization.cpp
    src/planner/viewpoint_planner_graph.cpp
    src/planner/viewpoint_planner_graph.hxx
    src/planner/viewpoint_planner_path.cpp
    src/planner/viewpoint_planner_path_tour.cpp
    src/planner/viewpoint_planner_sparse_matching.cpp
    src/planner/viewpoint_planner_data.h
    src/planner/viewpoint_planner_data.cpp
    src/planner/viewpoint_planner_opengl.cpp
    src/planner/viewpoint_planner_dump.cpp
    src/planner/motion_planner.h
    # Rendering
    src/rendering/octree_drawer.h
    src/rendering/octree_drawer.cpp
    src/rendering/voxel_drawer.h
    src/rendering/voxel_drawer.cpp
    # Reconstruction
    src/reconstruction/sparse_reconstruction.h
    src/reconstruction/sparse_reconstruction.cpp
    src/reconstruction/dense_reconstruction.h
    src/reconstruction/dense_reconstruction.cpp
)
cuda_add_library(viewpoint_planner_common_objects
    ${VIEWPOINT_PLANNER_SOURCES_COMMON}
)
if(WITH_OPENGL_OFFSCREEN)
    target_link_libraries(viewpoint_planner_common_objects Qt5::Core Qt5::Gui Qt5::OpenGL)
    target_link_libraries(viewpoint_planner_common_objects
      ${OMPL_LIBRARIES}
      ${Boost_LIBRARIES}
    )
endif()

cuda_add_executable(clip_point_cloud WIN32
    # Executable
    src/exe/clip_point_cloud.cpp
    # BH
    ../src/bh/utilities.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(clip_point_cloud
    viewpoint_planner_common_objects
)
target_link_libraries(clip_point_cloud
    ${Boost_LIBRARIES}
)

cuda_add_executable(compute_ground_truth_mesh WIN32
    # Executable
    src/exe/compute_ground_truth_mesh.cpp
    src/octree/occupancy_map.cpp
    # mLib
    src/mLib/mLib.h
    src/mLib/mLib.cpp
)
target_link_libraries(compute_ground_truth_mesh
    viewpoint_planner_common_objects
)
target_link_libraries(compute_ground_truth_mesh
    ${OCTOMAP_LIBRARIES}
    ${OCTOVIS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
if(WITH_OPENGL_OFFSCREEN)
    # Needs Qt headers when including viewpoint_planner.h
    target_link_libraries(compute_ground_truth_mesh
        ${OPENGL_LIBRARIES}
    )
    target_link_libraries(compute_ground_truth_mesh Qt5::Core Qt5::Gui Qt5::OpenGL)
endif()

cuda_add_executable(create_baseline_viewpoint_path
    # Executable
    src/exe/create_baseline_viewpoint_path.cpp
)
target_link_libraries(create_baseline_viewpoint_path
    viewpoint_planner_common_objects
)
target_link_libraries(create_baseline_viewpoint_path
    ${OCTOMAP_LIBRARIES}
    ${OCTOVIS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
if(WITH_OPENGL_OFFSCREEN)
    # Needs Qt headers when including viewpoint_planner.h
    target_link_libraries(create_baseline_viewpoint_path
        ${OPENGL_LIBRARIES}
    )
    target_link_libraries(create_baseline_viewpoint_path Qt5::Core Qt5::Gui Qt5::OpenGL)
endif()

cuda_add_executable(viewpoint_planner_cmdline WIN32
    # Executable
    src/exe/viewpoint_planner_cmdline.cpp
    # Offscreen rendering resources
    src/shaders_offscreen.qrc
)
target_link_libraries(viewpoint_planner_cmdline
    viewpoint_planner_common_objects
)
target_link_libraries(viewpoint_planner_cmdline
    ${OCTOMAP_LIBRARIES}
    ${OCTOVIS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
if(WITH_OPENGL_OFFSCREEN)
    target_link_libraries(viewpoint_planner_cmdline
        ${OPENGL_LIBRARIES}
    )
    target_link_libraries(viewpoint_planner_cmdline Qt5::Core Qt5::Gui Qt5::OpenGL)
endif()

QT5_WRAP_UI(viewpoint_planner_gui_UIS_H 
    src/ui/viewer_info_panel.ui
    src/ui/viewer_settings_panel.ui
    src/ui/viewer_planner_panel.ui
)

set(VIEWPOINT_PLANNER_SOURCES_GUI
    # UI
    src/ui/viewer_window.h
    src/ui/viewer_window.cpp
    src/ui/viewer_widget.h
    src/ui/viewer_widget.cpp
    src/ui/viewer_info_panel.h
    src/ui/viewer_settings_panel.h
    src/ui/viewer_planner_panel.h
    # Web
    src/web/web_socket_server.h
    src/web/web_socket_server.cpp
    # Rendering
    src/rendering/binned_octree_drawer.h
    src/rendering/binned_octree_drawer.cpp
    src/rendering/sparse_reconstruction_drawer.h
    src/rendering/sparse_reconstruction_drawer.cpp
    src/rendering/viewpoint_drawer.h
    src/rendering/viewpoint_drawer.hxx
)
cuda_add_library(viewpoint_planner_gui_objects
    ${VIEWPOINT_PLANNER_SOURCES_GUI}
    # UI
    ${viewpoint_planner_gui_UIS_H}
)
target_link_libraries(viewpoint_planner_gui_objects Qt5::Core Qt5::Xml Qt5::Gui Qt5::OpenGL Qt5::WebSockets)
target_link_libraries(viewpoint_planner_gui_objects
    viewpoint_planner_common_objects
    ${Boost_LIBRARIES}
)

cuda_add_executable(viewpoint_planner_gui WIN32
    # Executable
    src/exe/viewpoint_planner_gui.cpp
    # Resources
    src/shaders.qrc
)
target_link_libraries(viewpoint_planner_gui
    viewpoint_planner_common_objects
    viewpoint_planner_gui_objects
)
target_link_libraries(viewpoint_planner_gui
    ${OCTOMAP_LIBRARIES}
    ${OCTOVIS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GPERFTOOLS_LIBRARIES}
)
target_link_libraries(viewpoint_planner_gui Qt5::Core Qt5::Xml Qt5::Gui Qt5::OpenGL Qt5::WebSockets)
#target_link_libraries(viewpoint_planner_gui /home/bhepp/Projects/Libraries/libQGLViewer-2.6.4/lib/libQGLViewer-qt5.so)

if(WITH_TESTING)
    add_subdirectory(test)
endif()
