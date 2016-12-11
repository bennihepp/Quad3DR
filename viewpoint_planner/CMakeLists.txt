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
  include/
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(depth_octomap
  src/depth_octomap.cpp
  ../src/mLib.cpp
)
target_link_libraries(depth_octomap
  ${OCTOMAP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

QT5_WRAP_UI(viewpoint_planner_UIS_H src/viewer_settings_panel.ui src/viewer_info_panel.ui)
add_executable(viewpoint_planner WIN32
  src/viewpoint_planner_app.cpp
  ../src/mLib.cpp
  ../src/utilities.cpp
  src/viewer_window.h
  src/viewer_window.cpp
  src/viewer_widget.h
  src/viewer_widget.cpp
  src/viewer_info_panel.h
  src/viewer_settings_panel.h
  src/octree_drawer.h
  src/octree_drawer.cpp
  src/scene_object.h
  src/scene_object.cpp
  ${viewpoint_planner_UIS_H}
#  src/octomap_renderer.h
#  src/octomap_renderer.cpp
  src/shaders.qrc
)
target_link_libraries(viewpoint_planner
  ${OCTOMAP_LIBRARIES}
  ${OCTOVIS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
#target_link_libraries(viewpoint_planner Qt4::QtCore Qt4::QtXml Qt4::QtGui Qt4::QtOpenGL)
target_link_libraries(viewpoint_planner Qt5::Core Qt5::Xml Qt5::Gui Qt5::OpenGL)
#target_link_libraries(viewpoint_planner /home/bhepp/Projects/Libraries/libQGLViewer-2.6.4/lib/libQGLViewer-qt5.so)
