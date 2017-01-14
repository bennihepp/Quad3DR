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

# Qt4 was built with -reduce-relocations.
if(Qt5_POSITION_INDEPENDENT_CODE)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

include_directories(
	../external/QGLViewer
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)

add_executable(occupancy_map_from_sens
  # Executable
  src/exe/occupancy_map_from_sens.cpp
  # AIT
  ../src/common.cpp
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
  # AIT
  ../src/common.cpp
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

set(VIEWPOINT_PLANNER_SOURCES_COMMON
  # AIT
  ../src/common.cpp
  ../src/utilities.cpp
  # mLib
  src/mLib/mLib.h
  src/mLib/mLib.cpp
  # BVH
  src/bvh/bvh.h
  # Graph
  #src/graph/graph.h
  src/graph/graph_boost.h
  # Octree
  src/octree/occupancy_map.h
  src/octree/occupancy_map.hxx
  src/octree/occupancy_map_tree_navigator.hxx
  src/octree/occupancy_map.cpp
  src/octree/occupancy_node.h
  src/octree/occupancy_node.cpp
  # Planner
  src/planner/viewpoint.h
  src/planner/viewpoint.cpp
  src/planner/viewpoint_planner.h
  src/planner/viewpoint_planner.cpp
  src/planner/viewpoint_planner.hxx
  src/planner/viewpoint_planner_graph.cpp
  src/planner/viewpoint_planner_graph.hxx
  src/planner/viewpoint_planner_data.h
  src/planner/viewpoint_planner_data.cpp
  src/planner/motion_planner.h
  # Reconstruction
  src/reconstruction/sparse_reconstruction.h
  src/reconstruction/sparse_reconstruction.cpp
  src/reconstruction/dense_reconstruction.h
  src/reconstruction/dense_reconstruction.cpp
)

add_executable(viewpoint_planner_cmdline WIN32
  # Executable
  src/exe/viewpoint_planner_cmdline.cpp
  ${VIEWPOINT_PLANNER_SOURCES_COMMON}
)
target_link_libraries(viewpoint_planner_cmdline
  ${OCTOMAP_LIBRARIES}
  ${OCTOVIS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_executable(clip_point_cloud WIN32
  # Executable
  src/exe/clip_point_cloud.cpp
  # AIT
  ../src/common.cpp
  ../src/utilities.cpp
  # mLib
  src/mLib/mLib.h
  src/mLib/mLib.cpp
)
target_link_libraries(clip_point_cloud
  ${Boost_LIBRARIES}
)

QT5_WRAP_UI(viewpoint_planner_gui_UIS_H 
	src/ui/viewer_info_panel.ui
	src/ui/viewer_settings_panel.ui
	src/ui/viewer_planner_panel.ui
)

add_executable(viewpoint_planner_gui WIN32
  # Executable
  src/exe/viewpoint_planner_gui.cpp
  # UI
  src/ui/viewer_window.h
  src/ui/viewer_window.cpp
  src/ui/viewer_widget.h
  src/ui/viewer_widget.cpp
  src/ui/viewer_info_panel.h
  src/ui/viewer_settings_panel.h
  src/ui/viewer_planner_panel.h
  # Rendering
  src/rendering/octree_drawer.h
  src/rendering/octree_drawer.cpp
  src/rendering/voxel_drawer.h
  src/rendering/voxel_drawer.cpp
  src/rendering/sparse_reconstruction_drawer.h
  src/rendering/sparse_reconstruction_drawer.cpp
  src/rendering/viewpoint_drawer.h
  src/rendering/viewpoint_drawer.hxx
  ${viewpoint_planner_gui_UIS_H}
  src/shaders.qrc
  ${VIEWPOINT_PLANNER_SOURCES_COMMON}
)
target_link_libraries(viewpoint_planner_gui
  ${OCTOMAP_LIBRARIES}
  ${OCTOVIS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OMPL_LIBRARIES}
)
#target_link_libraries(viewpoint_planner_gui Qt4::QtCore Qt4::QtXml Qt4::QtGui Qt4::QtOpenGL)
target_link_libraries(viewpoint_planner_gui Qt5::Core Qt5::Xml Qt5::Gui Qt5::OpenGL)
#target_link_libraries(viewpoint_planner_gui /home/bhepp/Projects/Libraries/libQGLViewer-2.6.4/lib/libQGLViewer-qt5.so)
