//==================================================
// viewer_widget.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================

// This file is adapted from OctoMap.
// Original Copyright notice.
/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include "viewer_widget.h"
#include <cmath>
#include <algorithm>
#include <boost/assign.hpp>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <manipulatedCameraFrame.h>
#include <ait/qt_utils.h>
#include <ait/utilities.h>
#include <ait/pose.h>
#include <ait/color.h>

using namespace std;

ViewerWidget::ViewerWidget(const Options& options, const QGLFormat& format,
    ViewpointPlanner* planner, ViewerSettingsPanel* settings_panel,
    ViewerPlannerPanel* planner_panel, QWidget *parent)
    : QGLViewer(format, parent),
      options_(options),
      web_socket_server_(nullptr),
      custom_camera_(0.5, 1e5),
      z_near_coefficient_(0.01),
      planner_(planner), initialized_(false),
      settings_panel_(settings_panel), planner_panel_(planner_panel),
      octree_(nullptr), sparse_recon_(nullptr),
      dense_points_(nullptr), dense_points_size_(1),
      poisson_mesh_(nullptr),
      aspect_ratio_(-1),
      bbox_line_width_(5),
      viewpoint_motion_line_width_(5),
      min_information_filter_(0),
      viewpoint_color_mode_(Fixed),
      viewpoint_selected_component_(-1),
      planner_thread_(planner, this),
      viewpoint_path_branch_index_(0) {
    QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    policy.setHeightForWidth(true);
    setSizePolicy(policy);

    // Connect signals for settings panel
    connect(settings_panel_, SIGNAL(drawOctreeChanged(bool)), this, SLOT(setDrawOctree(bool)));
    connect(settings_panel_, SIGNAL(occupancyBinThresholdChanged(double)), this, SLOT(setOccupancyBinThreshold(double)));
    connect(settings_panel_, SIGNAL(colorFlagsChanged(uint32_t)), this, SLOT(setColorFlags(uint32_t)));
    connect(settings_panel_, SIGNAL(voxelAlphaChanged(double)), this, SLOT(setVoxelAlpha(double)));
    connect(settings_panel_, SIGNAL(drawFreeVoxelsChanged(bool)), this, SLOT(setDrawFreeVoxels(bool)));
    connect(settings_panel_, SIGNAL(drawAxesChanged(bool)), this, SLOT(setDrawAxes(bool)));
    connect(settings_panel_, SIGNAL(drawSingleBinChanged(bool)), this, SLOT(setDrawSingleBin(bool)));
    connect(settings_panel_, SIGNAL(drawCamerasChanged(bool)), this, SLOT(setDrawCameras(bool)));
    connect(settings_panel_, SIGNAL(drawSparsePointsChanged(bool)), this, SLOT(setDrawSparsePoints(bool)));
    connect(settings_panel_, SIGNAL(drawDensePointsChanged(bool)), this, SLOT(setDrawDensePoints(bool)));
    connect(settings_panel_, SIGNAL(drawPoissonMeshChanged(bool)), this, SLOT(setDrawPoissonMesh(bool)));
    connect(settings_panel_, SIGNAL(drawRegionOfInterestChanged(bool)), this, SLOT(setDrawRegionOfInterest(bool)));
    connect(settings_panel_, SIGNAL(drawBvhBboxChanged(bool)), this, SLOT(setDrawBvhBbox(bool)));
    connect(settings_panel_, SIGNAL(refreshTree(void)), this, SLOT(refreshTree(void)));
    connect(settings_panel_, SIGNAL(drawRaycastChanged(bool)), this, SLOT(setDrawRaycast(bool)));
    connect(settings_panel_, SIGNAL(captureRaycast(void)), this, SLOT(captureRaycast(void)));
    connect(settings_panel_, SIGNAL(useDroneCameraChanged(bool)), this, SLOT(setUseDroneCamera(bool)));
    connect(settings_panel_, SIGNAL(imagePoseChanged(ImageId)), this, SLOT(setImagePoseIndex(ImageId)));
    connect(settings_panel_, SIGNAL(minOccupancyChanged(double)), this, SLOT(setMinOccupancy(double)));
    connect(settings_panel_, SIGNAL(maxOccupancyChanged(double)), this, SLOT(setMaxOccupancy(double)));
    connect(settings_panel_, SIGNAL(minObservationsChanged(uint32_t)), this, SLOT(setMinObservations(uint32_t)));
    connect(settings_panel_, SIGNAL(maxObservationsChanged(uint32_t)), this, SLOT(setMaxObservations(uint32_t)));
    connect(settings_panel_, SIGNAL(minVoxelSizeChanged(double)), this, SLOT(setMinVoxelSize(double)));
    connect(settings_panel_, SIGNAL(maxVoxelSizeChanged(double)), this, SLOT(setMaxVoxelSize(double)));
    connect(settings_panel_, SIGNAL(minWeightChanged(double)), this, SLOT(setMinWeight(double)));
    connect(settings_panel_, SIGNAL(maxWeightChanged(double)), this, SLOT(setMaxWeight(double)));
    connect(settings_panel_, SIGNAL(minInformationChanged(double)), this, SLOT(setMinInformation(double)));
    connect(settings_panel_, SIGNAL(maxInformationChanged(double)), this, SLOT(setMaxInformation(double)));
    connect(settings_panel_, SIGNAL(renderTreeDepthChanged(std::size_t)), this, SLOT(setRenderTreeDepth(std::size_t)));
    connect(settings_panel_, SIGNAL(renderObservationThresholdChanged(std::size_t)), this, SLOT(setRenderObservationThreshold(std::size_t)));

    connect(planner_panel_, SIGNAL(pauseContinueViewpointGraph()), this, SLOT(pauseContinueViewpointGraph()));
    connect(planner_panel_, SIGNAL(pauseContinueViewpointMotions()), this, SLOT(pauseContinueViewpointMotions()));
    connect(planner_panel_, SIGNAL(pauseContinueViewpointPath()), this, SLOT(pauseContinueViewpointPath()));
    connect(planner_panel_, SIGNAL(solveViewpointTSP()), this, SLOT(solveViewpointTSP()));
    connect(planner_panel_, SIGNAL(resetViewpoints()), this, SLOT(resetViewpoints()));
    connect(planner_panel_, SIGNAL(resetViewpointMotions()), this, SLOT(resetViewpointMotions()));
    connect(planner_panel_, SIGNAL(resetViewpointPath()), this, SLOT(resetViewpointPath()));
    connect(planner_panel_, SIGNAL(saveViewpointGraph(const std::string&)), this, SLOT(onSaveViewpointGraph(const std::string&)));
    connect(planner_panel_, SIGNAL(exportViewpointPathAsJson(const std::string&)), this, SLOT(onExportViewpointPathAsJson(const std::string&)));
    connect(planner_panel_, SIGNAL(loadViewpointGraph(const std::string&)), this, SLOT(onLoadViewpointGraph(const std::string&)));
    connect(planner_panel_, SIGNAL(saveViewpointPath(const std::string&)), this, SLOT(onSaveViewpointPath(const std::string&)));
    connect(planner_panel_, SIGNAL(loadViewpointPath(const std::string&)), this, SLOT(onLoadViewpointPath(const std::string&)));
    connect(planner_panel, SIGNAL(drawViewpointGraphChanged(bool)), this, SLOT(setDrawViewpointGraph(bool)));
    connect(planner_panel, SIGNAL(viewpointGraphSelectionChanged(std::size_t)), this, SLOT(setViewpointGraphSelectionIndex(std::size_t)));
    connect(planner_panel, SIGNAL(drawViewpointMotionsChanged(bool)), this, SLOT(setDrawViewpointMotions(bool)));
    connect(planner_panel, SIGNAL(drawViewpointPathChanged(bool)), this, SLOT(setDrawViewpointPath(bool)));
    connect(planner_panel, SIGNAL(viewpointPathBranchSelectionChanged(std::size_t)), this, SLOT(setViewpointPathBranchSelectionIndex(std::size_t)));
    connect(planner_panel, SIGNAL(viewpointPathSelectionChanged(std::size_t)), this, SLOT(setViewpointPathSelectionIndex(std::size_t)));
    connect(planner_panel, SIGNAL(useFixedColorsChanged(bool)), this, SLOT(setUseFixedColors(bool)));
    connect(planner_panel, SIGNAL(alphaParameterChanged(double)), this, SLOT(setAlphaParameter(double)));
    connect(planner_panel, SIGNAL(betaParameterChanged(double)), this, SLOT(setBetaParameter(double)));
    connect(planner_panel, SIGNAL(minInformationFilterChanged(double)), this, SLOT(setMinInformationFilter(double)));
    connect(planner_panel, SIGNAL(viewpointPathLineWidthChanged(double)), this, SLOT(setViewpointPathLineWidth(double)));
    connect(planner_panel, SIGNAL(viewpointColorModeChanged(std::size_t)), this, SLOT(setViewpointColorMode(std::size_t)));
    connect(planner_panel, SIGNAL(viewpointGraphComponentChanged(int)), this, SLOT(setViewpointGraphComponent(int)));

    // Fill occupancy dropbox in settings panel
    FloatType selected_occupancy_bin_threshold = octree_drawer_.getOccupancyBinThreshold();
    settings_panel_->initializeOccupancyBinThresholds(octree_drawer_.getOccupancyBins());
    settings_panel_->selectOccupancyBinThreshold(selected_occupancy_bin_threshold);

    std::vector<std::pair<std::string, uint32_t>> color_flags_uint;
    for (const auto& entry : VoxelDrawer::getAvailableColorFlags()) {
      color_flags_uint.push_back(std::make_pair(entry.first, static_cast<uint32_t>(entry.second)));
    }
    AIT_ASSERT(!color_flags_uint.empty());
    settings_panel_->initializeColorFlags(color_flags_uint);
    settings_panel_->selectColorFlags(color_flags_uint[0].second);

    octree_drawer_.setMinOccupancy(settings_panel_->getMinOccupancy());
    octree_drawer_.setMaxOccupancy(settings_panel_->getMaxOccupancy());
    octree_drawer_.setMinObservations(settings_panel_->getMinObservations());
    octree_drawer_.setMaxObservations(settings_panel_->getMaxObservations());
    octree_drawer_.setMinVoxelSize(settings_panel_->getMinVoxelSize());
    octree_drawer_.setMaxVoxelSize(settings_panel_->getMaxVoxelSize());
    octree_drawer_.setRenderTreeDepth(settings_panel_->getRenderTreeDepth());
    octree_drawer_.setRenderObservationThreshold(settings_panel_->getRenderObservationThreshold());
    octree_drawer_.setMinWeight(settings_panel_->getMinWeight());
    octree_drawer_.setMaxWeight(settings_panel_->getMaxWeight());
    octree_drawer_.setMinInformation(settings_panel_->getMinInformation());
    octree_drawer_.setMaxInformation(settings_panel_->getMaxInformation());

    connect(this, SIGNAL(viewpointsChanged()), this, SLOT(updateViewpoints()));
    connect(this, SIGNAL(plannerThreadPaused()), this, SLOT(onPlannerThreadPaused()));

    std::vector<std::pair<std::string, std::size_t>> color_mode_size_t;
    for (const auto& entry : getAvailableViewpointColorModes()) {
      color_mode_size_t.push_back(std::make_pair(entry.first, static_cast<std::size_t>(entry.second)));
    }
    AIT_ASSERT(!color_mode_size_t.empty());
    planner_panel_->initializeViewpointColorMode(color_mode_size_t);
    planner_panel_->selectViewpointColorMode(color_mode_size_t[0].second);

    planner_thread_.setAlpha(planner_panel_->getAlphaParameter());
    planner_thread_.setBeta(planner_panel_->getBetaParameter());
    planner_thread_.setOperation(ViewpointPlannerThread::Operation::VIEWPOINT_UPDATE);
    planner_thread_.start();
    continuePlannerThread();

    if (options_.websocket_enable) {
      web_socket_server_ = new WebSocketServer(options_.websocket_port, this);
    }
}

ViewerWidget::~ViewerWidget() {
  planner_thread_.finish();
}

std::vector<std::pair<std::string, ViewerWidget::ViewpointColorMode>> ViewerWidget::getAvailableViewpointColorModes() const {
  std::vector<std::pair<std::string, ViewpointColorMode>> modes = boost::assign::pair_list_of
      ("Fixed", ViewpointColorMode::Fixed)
      ("Component", ViewpointColorMode::Component)
      ("Information", ViewpointColorMode::Information);
  return modes;
}

void ViewerWidget::init() {
  setCamera(&custom_camera_);

  initialized_ = true;

  //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::AltModifier);
  //    setHandlerKeyboardModifiers(QGLViewer::FRAME, Qt::NoModifier);
  //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::ControlModifier);
  setMouseTracking(true);

  // Restore previous viewer state.
  restoreStateFromFile();
  std::cout << "QGLViewer.stateFilename: " <<stateFileName().toStdString() << std::endl;

  // Make camera the default manipulated frame.
  setManipulatedFrame(camera()->frame());
  // invert mousewheel (more like Blender)
  camera()->frame()->setWheelSensitivity(-0.5);

  // TODO
  camera()->setSceneCenter(qglviewer::Vec(0, 0, 0));
  camera()->setSceneRadius(50);

  camera()->setPosition(qglviewer::Vec(0, 0, 100));
  camera()->lookAt(qglviewer::Vec(0, 0, 0));

  setUseDroneCamera(settings_panel_->getUseDroneCamera());

  // background color defaults to white
  this->setBackgroundColor(QColor(255,255,255));
  this->qglClearColor(this->backgroundColor());

  initAxesDrawer();

  std::cout << "zNear: " << camera()->zNear() << ", zFar: " << camera()->zFar() << std::endl;
  // TODO: Hack, make this dependent on the scene
//  camera()->setZNearCoefficient(z_near_coefficient_);
//  camera()->setZClippingCoefficient(1);
  sparce_recon_drawer_.init();
  dense_points_drawer_.init();
  dense_points_drawer_.setDrawPoints(false);
  poisson_mesh_drawer_.init();
  poisson_mesh_drawer_.setDrawTriangles(false);
  region_of_interest_drawer_.init();
  region_of_interest_drawer_.setDrawLines(false);
  bvh_bbox_drawer_.init();
  bvh_bbox_drawer_.setDrawLines(false);
  viewpoint_path_drawer_.init();
  viewpoint_graph_drawer_.init();
  viewpoint_motion_line_drawer_.init();
  if (octree_ != nullptr) {
    showOctree(octree_);
  }
  if (sparse_recon_ != nullptr) {
    showSparseReconstruction(sparse_recon_);
  }
  if (dense_points_ != nullptr) {
    showDensePoints(dense_points_);
  }
  if (poisson_mesh_ != nullptr) {
    showPoissonMesh(poisson_mesh_);
  }
  showRegionOfInterest(planner_->getRoi());
  showBvhBbox(planner_->getBvhBbox());
}

void ViewerWidget::initAxesDrawer() {
  axes_drawer_.init();
  std::vector<OGLLineData> line_data;
  FloatType axes_length = 1;
  OGLVertexData axes_origin(0, 0, 0);
  OGLVertexData axes_end_x(axes_origin);
  axes_end_x.x += axes_length;
  OGLVertexData axes_end_y(axes_origin);
  axes_end_y.y += axes_length;
  OGLVertexData axes_end_z(axes_origin);
  axes_end_z.z += axes_length;
  OGLColorData color_x(1, 0, 0, 1);
  OGLColorData color_y(0, 1, 0, 1);
  OGLColorData color_z(0, 0, 1, 1);
  line_data.emplace_back(OGLVertexDataRGBA(axes_origin, color_x), OGLVertexDataRGBA(axes_end_x, color_x));
  line_data.emplace_back(OGLVertexDataRGBA(axes_origin, color_y), OGLVertexDataRGBA(axes_end_y, color_y));
  line_data.emplace_back(OGLVertexDataRGBA(axes_origin, color_z), OGLVertexDataRGBA(axes_end_z, color_z));
  axes_drawer_.upload(line_data);
}

int ViewerWidget::heightForWidth(int w) const {
    if (aspect_ratio_ <= 0) {
        return -1;
    }
    return static_cast<int>(w / aspect_ratio_);
}

QSize ViewerWidget::sizeHint() const {
    if (aspect_ratio_ <= 0) {
        return QSize();
    }
    return QSize(width(), width() / aspect_ratio_);
}

void ViewerWidget::showOctree(const ViewpointPlanner::OccupancyMapType* octree) {
    octree_ = octree;
    if (!initialized_) {
        return;
    }

    // update viewer stat
    double minX, minY, minZ, maxX, maxY, maxZ;
    minX = minY = minZ = -10; // min bbx for drawing
    maxX = maxY = maxZ = 10;  // max bbx for drawing
//    double sizeX, sizeY, sizeZ;
//    sizeX = sizeY = sizeZ = 0.;
    size_t memoryUsage = 0;
    size_t num_nodes = 0;

    ait::Timer timer;
    // get map bbx
//    double lminX, lminY, lminZ, lmaxX, lmaxY, lmaxZ;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (it->getObservationCount() == 0) {
        continue;
      }
      minX = std::min(minX, it.getX());
      maxX = std::max(maxX, it.getX());
      minY = std::min(minY, it.getY());
      maxY = std::max(maxY, it.getY());
      minZ = std::min(minZ, it.getZ());
      maxZ = std::max(maxZ, it.getZ());
    }
//    octree_->getMetricMin(lminX, lminY, lminZ);
//    octree_->getMetricMax(lmaxX, lmaxY, lmaxZ);
//    // transform to world coords using map origin
//    octomap::point3d pmin(lminX, lminY, lminZ);
//    octomap::point3d pmax(lmaxX, lmaxY, lmaxZ);
//    lminX = pmin.x(); lminY = pmin.y(); lminZ = pmin.z();
//    lmaxX = pmax.x(); lmaxY = pmax.y(); lmaxZ = pmax.z();
//    // update global bbx
//    if (lminX < minX) minX = lminX;
//    if (lminY < minY) minY = lminY;
//    if (lminZ < minZ) minZ = lminZ;
//    if (lmaxX > maxX) maxX = lmaxX;
//    if (lmaxY > maxY) maxY = lmaxY;
//    if (lmaxZ > maxZ) maxZ = lmaxZ;
//    double lsizeX, lsizeY, lsizeZ;
//    // update map stats
//    octree_->getMetricSize(lsizeX, lsizeY, lsizeZ);
//    if (lsizeX > sizeX) sizeX = lsizeX;
//    if (lsizeY > sizeY) sizeY = lsizeY;
//    if (lsizeZ > sizeZ) sizeZ = lsizeZ;
    memoryUsage += octree_->memoryUsage();
    num_nodes += octree_->size();
    timer.printTiming("Computing octree metrics");
    std::cout << "Bounding box: [" << minX << ", " << minY << ", " << minZ << "] -> "
        << "[" << maxX << ", " << maxY << ", " << maxZ << "]" << std::endl;

    refreshTree();

    setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));
}

void ViewerWidget::showDensePoints(const ViewpointPlanner::PointCloudType* dense_points) {
  dense_points_ = dense_points;
  if (!initialized_) {
      return;
  }

  std::cout << "Showing dense points" << std::endl;
  std::vector<OGLVertexDataRGBA> point_data;
  point_data.reserve(dense_points_->m_points.size());
  for (std::size_t i = 0; i < dense_points_->m_points.size(); ++i) {
    const auto& ml_vertex = dense_points_->m_points[i];
    const auto& ml_color = dense_points_->m_colors[i];
    OGLVertexDataRGBA point;
    point.x = static_cast<float>(ml_vertex.x);
    point.y = static_cast<float>(ml_vertex.y);
    point.z = static_cast<float>(ml_vertex.z);
    point.r = ml_color.r;
    point.g = ml_color.g;
    point.b = ml_color.b;
    point.a = 1;
    point_data.push_back(point);
  }
//  // Debugging code for visualizing ROI
//  const auto bbox = planner_->getRoiBbox();
//  for (std::size_t ix = 0; ix <= 100; ++ix) {
//    for (std::size_t iy = 0; iy <= 100; ++iy) {
//      for (std::size_t iz = 0; iz <= 100; ++iz) {
//        OGLVertexDataRGBA point;
//        const auto extent = bbox.getExtent();
//        point.x = ix * extent(0) / 100 + bbox.getMinimum(0);
//        point.y = iy * extent(1) / 100 + bbox.getMinimum(1);
//        point.z = iz * extent(2) / 100 + bbox.getMinimum(2);
//        point.r = 1;
//        point.g = 0;
//        point.b = 0;
//        point.a = 1;
//        point_data.push_back(point);
//      }
//    }
//  }
//  // End of debuggin code
//  // Debugging code for visualizing BVH bounding box
//  const auto bbox = planner_->getBvhTree().getRoot()->getBoundingBox();
//  for (std::size_t ix = 0; ix <= 100; ++ix) {
//    for (std::size_t iy = 0; iy <= 100; ++iy) {
//      for (std::size_t iz = 0; iz <= 100; ++iz) {
//        OGLVertexDataRGBA point;
//        const auto extent = bbox.getExtent();
//        point.x = ix * extent(0) / 100 + bbox.getMinimum(0);
//        point.y = iy * extent(1) / 100 + bbox.getMinimum(1);
//        point.z = iz * extent(2) / 100 + bbox.getMinimum(2);
//        point.r = 1;
//        point.g = 0;
//        point.b = 0;
//        point.a = 1;
//        point_data.push_back(point);
//      }
//    }
//  }
//  // End of debuggin code
  std::cout << "Uploading " << point_data.size() << " points" << std::endl;
  dense_points_drawer_.upload(point_data);
}

void ViewerWidget::showPoissonMesh(const ViewpointPlanner::MeshType* poisson_mesh) {
  poisson_mesh_ = poisson_mesh;
  if (!initialized_) {
      return;
  }

  std::cout << "Showing poisson_mesh" << std::endl;
  std::vector<OGLTriangleData> triangle_data;
  triangle_data.reserve(poisson_mesh_->m_FaceIndicesVertices.size());
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const ViewpointPlanner::MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    AIT_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
    const ml::vec3f& v1 = poisson_mesh_->m_Vertices[face[0]];
    const ml::vec3f& v2 = poisson_mesh_->m_Vertices[face[1]];
    const ml::vec3f& v3 = poisson_mesh_->m_Vertices[face[2]];
    const ml::vec4f& c1 = poisson_mesh_->m_Colors[face[0]];
    const ml::vec4f& c2 = poisson_mesh_->m_Colors[face[1]];
    const ml::vec4f& c3 = poisson_mesh_->m_Colors[face[2]];
    const FloatType a1 = 1;
    const FloatType a2 = 1;
    const FloatType a3 = 1;
    OGLTriangleData triangle;
    triangle.vertex1 = OGLVertexDataRGBA(v1.x, v1.y, v1.z, c1.r, c1.g, c1.b, a1);
    triangle.vertex2 = OGLVertexDataRGBA(v2.x, v2.y, v2.z, c2.r, c2.g, c2.b, a2);
    triangle.vertex3 = OGLVertexDataRGBA(v3.x, v3.y, v3.z, c3.r, c3.g, c3.b, a3);
    triangle_data.push_back(triangle);
  }
  std::cout << "Uploading " << triangle_data.size() << " triangles" << std::endl;
  poisson_mesh_drawer_.upload(triangle_data);
}

void ViewerWidget::showRegionOfInterest(const ViewpointPlanner::RegionType& roi) {
  std::cout << "Showing region of interest" << std::endl;
  std::vector<OGLLineData> line_data;
  const ViewpointPlanner::RegionType::Polygon2DType& polygon = roi.getPolygon2D();
  line_data.reserve(polygon.getVertexCount() * 3);
  const ait::Color4<FloatType> c(1, 1, 0, 1);
  for (size_t i = 0; i < polygon.getVertexCount(); ++i) {
    std::size_t prev_i;
    if (i == 0) {
      prev_i = polygon.getVertexCount() - 1;
    }
    else {
      prev_i = i - 1;
    }
    const auto v1 = polygon.getVertex(prev_i);
    const auto v2 = polygon.getVertex(i);
    OGLLineData line;
    line.vertex1 = OGLVertexDataRGBA(v1(0), v1(1), roi.getLowerPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(v2(0), v2(1), roi.getLowerPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
    line.vertex1 = OGLVertexDataRGBA(v1(0), v1(1), roi.getUpperPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(v2(0), v2(1), roi.getUpperPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
    line.vertex1 = OGLVertexDataRGBA(v1(0), v1(1), roi.getLowerPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(v1(0), v1(1), roi.getUpperPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
  }
  std::cout << "Uploading " << line_data.size() << " lines" << std::endl;
  region_of_interest_drawer_.upload(line_data);
}

void ViewerWidget::showBvhBbox(const ViewpointPlanner::BoundingBoxType& bvh_bbox) {
  std::cout << "Showing BVH bounding box" << std::endl;
  std::vector<OGLLineData> line_data;
  line_data.reserve(4 * 3);
  const ait::Color4<FloatType> c(1, 0, 0, 1);
  OGLLineData line;
  const Vector3 min = bvh_bbox.getMinimum();
  const Vector3 max = bvh_bbox.getMaximum();
  // Draw lower rectangle
  line.vertex1 = OGLVertexDataRGBA(min(0), min(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(max(0), min(1), min(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(max(0), min(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(max(0), max(1), min(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(max(0), max(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(min(0), max(1), min(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(min(0), max(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(min(0), min(1), min(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  // Draw upper rectangle
  line.vertex1 = OGLVertexDataRGBA(min(0), min(1), max(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(max(0), min(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(max(0), min(1), max(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(max(0), max(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(max(0), max(1), max(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(min(0), max(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(min(0), max(1), max(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(min(0), min(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  // Draw sides
  line.vertex1 = OGLVertexDataRGBA(min(0), min(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(min(0), min(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(max(0), min(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(max(0), min(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(min(0), max(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(min(0), max(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  line.vertex1 = OGLVertexDataRGBA(max(0), max(1), min(2), c.r(), c.g(), c.b(), c.a());
  line.vertex2 = OGLVertexDataRGBA(max(0), max(1), max(2), c.r(), c.g(), c.b(), c.a());
  line_data.push_back(line);
  std::cout << "Uploading " << line_data.size() << " lines" << std::endl;
  bvh_bbox_drawer_.upload(line_data);
}

void ViewerWidget::showViewpointGraph(const std::size_t selected_index /*= (std::size_t)-1*/) {
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  // Fill viewpoint graph dropbox in planner panel
  std::vector<std::pair<std::string, size_t>> viewpoint_graph_gui_entries;
  for (size_t i = 0; i < viewpoint_graph_copy_.size(); ++i) {
    ViewpointPlanner::FloatType total_information = std::get<2>(viewpoint_graph_copy_[i]);
    if (total_information < min_information_filter_) {
      continue;
    }
    std::string name = std::to_string(i) + " - " + std::to_string(total_information);
    viewpoint_graph_gui_entries.push_back(std::make_pair(name, i));
  }
  planner_panel_->initializeViewpointGraph(viewpoint_graph_gui_entries);

  // Fill connected component box
  const int viewpoint_component_selection = planner_panel_->getViewpointComponentSelection();
  std::vector<std::size_t> components;
  std::size_t num_components;
  std::tie(components, num_components) = planner_->getConnectedComponents();
  std::vector<std::pair<std::string, int>> viewpoint_component_gui_entries;
  viewpoint_component_gui_entries.push_back(std::make_pair("All", -1));
  for (size_t i = 0; i < num_components; ++i) {
    std::string name = std::to_string(i);
    viewpoint_component_gui_entries.push_back(std::make_pair(name, i));
  }
  planner_panel_->initializeViewpointComponents(viewpoint_component_gui_entries);
  if (viewpoint_component_selection < 0) {
    planner_panel_->setViewpointComponentSelectionByItemIndex(0);
  }

  // Compute min-max of information value
  ait::MinMaxTracker<FloatType> min_max;
  for (const auto& entry : viewpoint_graph_copy_) {
    FloatType total_information = std::get<2>(entry);
    min_max.update(total_information);
  }

  // Debugging code
//  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
//  const bool valid_selection = selected_index != (std::size_t)-1;
//  const bool use_fixed_colors = planner_panel_->isUseFixedColorsChecked();
//  if (inspect_graph_motions && valid_selection) {
//    std::cout << "Viewpoint " << selected_index << " has "
//        << planner_->getViewpointGraph().numOutEdgesByNode(selected_index) << " edges" << std::endl;
//    auto edges = planner_->getViewpointGraph().getEdgesByNode(selected_index);
//    for (auto it = edges.begin(); it != edges.end(); ++it) {
//      std::cout << "  edge from " << it.sourceNode() << " to " << it.targetNode() << " weight " << it.weight() << std::endl;
//    }
//  }

  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  const bool valid_selection = selected_index != (std::size_t)-1;
  // Make a copy of the graph poses so that we can later on access
  // viewpoints select by the user
  std::vector<Pose> poses;
  std::vector<typename ViewpointDrawer<FloatType>::Color4> colors;
  const ait::ColorMapHot<FloatType> cmap;
  for (const auto& entry : viewpoint_graph_copy_) {
    if (viewpoint_selected_component_ >= 0 && components[std::get<0>(entry)] != (std::size_t)viewpoint_selected_component_) {
      continue;
    }
    FloatType total_information = std::get<2>(entry);
    if (inspect_graph_motions && valid_selection) {
      const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(entry);
      if (viewpoint_index != selected_index && !planner_->getViewpointGraph().getEdgesByNode(selected_index).containsTargetNode(viewpoint_index)) {
        continue;
      }
    }
    const Pose& pose = std::get<1>(entry);
    typename ViewpointDrawer<FloatType>::Color4 color;
    if (viewpoint_color_mode_ == Fixed
        && valid_selection && std::get<0>(entry) == std::get<0>(viewpoint_graph_copy_[selected_index])) {
      color = typename ViewpointDrawer<FloatType>::Color4(0.8f, 0.0f, 0.0f, 0.6f);
    }
    else if (viewpoint_color_mode_ == Fixed) {
      color = typename ViewpointDrawer<FloatType>::Color4(0.7f, 0.8f, 0.0f, 0.6f);
    }
    else if (viewpoint_color_mode_ == Component) {
      const std::size_t component = components[std::get<0>(entry)];
      const FloatType value = ait::normalize<FloatType>(component, 0, num_components);
      color = cmap.map(value, 0.6f);
    }
    else {
      const FloatType value = min_max.normalize(total_information);
      color = cmap.map(value, 0.6f);
    }
    poses.push_back(pose);
    colors.push_back(color);
  }
  // Draw viewpoint camera frustums
  viewpoint_graph_drawer_.setCamera(sparse_recon_->getCameras().cbegin()->second);
  viewpoint_graph_drawer_.setViewpoints(poses, colors);

  planner_panel_->setViewpointGraphSize(viewpoint_graph_copy_.size());
  planner_panel_->setViewpointMotionsSize(planner_->getViewpointGraph().numEdges());
  lock.unlock();

  if (inspect_graph_motions && valid_selection) {
    showViewpointGraphMotions(selected_index);
  }

  update();
}

void ViewerWidget::showViewpointPath(const std::size_t selected_index /*= (std::size_t)-1*/) {
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  // Fill viewpoint path branch dropbox in planner panel
  std::vector<std::pair<std::string, size_t>> viewpoint_path_branch_gui_entries;
  for (size_t i = 0; i < planner_->getViewpointPaths().size(); ++i) {
    ViewpointPlanner::FloatType objective = planner_->getViewpointPaths()[i].acc_objective;
    std::string name = std::to_string(i) + " - " + std::to_string(objective);
    viewpoint_path_branch_gui_entries.push_back(std::make_pair(name, i));
  }
  planner_panel_->initializeViewpointPathBranch(viewpoint_path_branch_gui_entries);

  // Fill viewpoint path dropbox in planner panel
  std::vector<std::pair<std::string, size_t>> viewpoint_path_gui_entries;
  for (size_t i = 0; i < viewpoint_path_copy_.size(); ++i) {
    ViewpointPlanner::FloatType information = std::get<2>(viewpoint_path_copy_[i]);
    std::string name = std::to_string(i) + " - " + std::to_string(information);
    viewpoint_path_gui_entries.push_back(std::make_pair(name, i));
  }
  planner_panel_->initializeViewpointPath(viewpoint_path_gui_entries);

  // Compute min-max of information value
  ait::MinMaxTracker<FloatType> min_max;
  for (const auto& entry : viewpoint_path_copy_) {
    const FloatType total_information = std::get<2>(entry);
    min_max.update(total_information);
  }

  const bool valid_selection = selected_index != (std::size_t)-1;
  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  // Generate OGL data for viewpoint drawer
  std::vector<Pose> poses;
  std::vector<typename ViewpointDrawer<FloatType>::Color4> colors;
  const ait::ColorMapHot<FloatType> cmap;
  ViewpointPlanner::ViewpointEntryIndex selected_viewpoint_index = (std::size_t)-1;
  if (valid_selection) {
    selected_viewpoint_index = std::get<0>(viewpoint_path_copy_[selected_index]);
  }
  for (const auto& entry : viewpoint_path_copy_) {
    const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(entry);
    FloatType total_information = std::get<2>(entry);
    if (inspect_graph_motions && valid_selection) {
      if (selected_viewpoint_index != viewpoint_index
          && !planner_->getViewpointGraph().getEdgesByNode(selected_viewpoint_index).containsTargetNode(viewpoint_index)) {
        continue;
      }
    }

    const Pose& pose = std::get<1>(entry);
    typename ViewpointDrawer<FloatType>::Color4 color;
    if (viewpoint_color_mode_ == Fixed) {
      if (valid_selection && viewpoint_index == selected_viewpoint_index) {
        color = typename ViewpointDrawer<FloatType>::Color4(0.8f, 0.0f, 0.0f, 0.6f);
      }
      else {
        color = typename ViewpointDrawer<FloatType>::Color4(0.0f, 0.1f, 0.8f, 0.6f);
      }
    }
    else if (viewpoint_color_mode_ == Component) {
      std::vector<std::size_t> component;
      std::size_t num_components;
      std::tie(component, num_components) = planner_->getConnectedComponents();
      std::size_t comp = component[viewpoint_index];
      const FloatType value = ait::normalize<FloatType>(comp, 0, num_components);
      color = cmap.map(value, 0.6f);
    }
    else {
      const FloatType value = min_max.normalize(total_information);
      color = cmap.map(value, 0.6f);
    }
    poses.push_back(pose);
    colors.push_back(color);
  }
  // Draw viewpoint camera frustums
  viewpoint_path_drawer_.setCamera(sparse_recon_->getCameras().cbegin()->second);
  viewpoint_path_drawer_.setViewpoints(poses, colors);

  planner_panel_->setViewpointPathSize(viewpoint_path_copy_.size());
  lock.unlock();

  showViewpointPathMotions(selected_index);

  update();
}

void ViewerWidget::showViewpointGraphMotions(const std::size_t selected_index) {
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  // Draw motion path
  std::vector<OGLLineData> lines;
//  const OGLColorData line_color(0.7f, 0.1f, 0.0f, 1.0f);
  const ait::ColorMapHot<FloatType> cmap;
  ViewpointPlanner::ViewpointGraph& viewpoint_graph = planner_->getViewpointGraph();
  auto edges = viewpoint_graph.getEdgesByNode(selected_index);
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    const ViewpointPlanner::ViewpointEntryIndex from_index = selected_index;
    const ViewpointPlanner::ViewpointEntryIndex to_index = it.targetNode();
    const ViewpointPlanner::Motion& motion = planner_->getViewpointMotion(from_index, to_index);
    // Careful, assuming random access iterators
    FloatType total_distance = 0;
    for (auto it2 = motion.poses.cbegin() + 1; it2 < motion.poses.cend(); ++it2) {
      const Vector3 from_pos = (it2 - 1)->getWorldPosition();
      const Vector3 to_pos = it2->getWorldPosition();
      total_distance += (from_pos - to_pos).norm();
    }
    FloatType acc_dist = 0;
    for (auto it2 = motion.poses.cbegin() + 1; it2 < motion.poses.cend(); ++it2) {
      const Vector3 from_pos = (it2 - 1)->getWorldPosition();
      const Vector3 to_pos = it2->getWorldPosition();
      const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
      const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
      const FloatType local_dist = (from_pos - to_pos).norm();
      const FloatType value1 = ait::normalize<FloatType>(acc_dist, 0, total_distance);
      const FloatType value2 = ait::normalize<FloatType>(acc_dist + local_dist, 0, total_distance);
      acc_dist += local_dist;
      ait::Color3<FloatType> color1 = cmap.map(value1);
      const OGLColorData line_color1(color1.r(), color1.g(), color1.b(), 1.0f);
      ait::Color3<FloatType> color2 = cmap.map(value2);
      const OGLColorData line_color2(color2.r(), color2.g(), color2.b(), 1.0f);
      lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color1), OGLVertexDataRGBA(vertex2, line_color2));
    }
  }
  std::cout << "Selected viewpoint " << selected_index << " has " << edges.size() << " edges" << std::endl;
  viewpoint_motion_line_drawer_.upload(lines);

  lock.unlock();

  update();
}

void ViewerWidget::showViewpointPathMotions(const std::size_t selected_index /*= (std::size_t)-1*/) {
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  const bool valid_selection = selected_index != (std::size_t)-1;
  const ait::ColorMapHot<FloatType> cmap;
  // Draw motion path
  std::vector<OGLLineData> lines;
  if (inspect_graph_motions && valid_selection) {
    const OGLColorData line_color(0.7f, 0.1f, 0.0f, 1.0f);
    const ViewpointPlanner::ViewpointPath viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index_];
    const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[selected_index].viewpoint_index;
    const ViewpointPlanner::ViewpointEntry viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
    auto edges = planner_->getViewpointGraph().getEdgesByNode(viewpoint_index);
    for (auto it = edges.begin(); it != edges.end(); ++it) {
      const ViewpointPlanner::ViewpointEntryIndex from_index = viewpoint_index;
      const ViewpointPlanner::ViewpointEntryIndex to_index = it.targetNode();
      const ViewpointPlanner::Motion& motion = planner_->getViewpointMotion(from_index, to_index);
      // Careful, assuming random access iterators
      FloatType total_distance = 0;
      for (auto it2 = motion.poses.cbegin() + 1; it2 < motion.poses.cend(); ++it2) {
        const Vector3 from_pos = (it2 - 1)->getWorldPosition();
        const Vector3 to_pos = it2->getWorldPosition();
        total_distance += (from_pos - to_pos).norm();
      }
      FloatType acc_dist = 0;
      for (auto it2 = motion.poses.cbegin() + 1; it2 < motion.poses.cend(); ++it2) {
        const Vector3 from_pos = (it2 - 1)->getWorldPosition();
        const Vector3 to_pos = it2->getWorldPosition();
        const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
        const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
        const FloatType local_dist = (from_pos - to_pos).norm();
        const FloatType value1 = ait::normalize<FloatType>(acc_dist, 0, total_distance);
        const FloatType value2 = ait::normalize<FloatType>(acc_dist + local_dist, 0, total_distance);
        acc_dist += local_dist;
        ait::Color3<FloatType> color1 = cmap.map(value1);
        const OGLColorData line_color1(color1.r(), color1.g(), color1.b(), 1.0f);
        ait::Color3<FloatType> color2 = cmap.map(value2);
        const OGLColorData line_color2(color2.r(), color2.g(), color2.b(), 1.0f);
        lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color1), OGLVertexDataRGBA(vertex2, line_color2));
      }
    }
  }
  else if (viewpoint_path_copy_.size() > 1) {
    // Compute total distance of path
    FloatType total_distance = 0;
    for (auto it = viewpoint_path_order_copy_.begin(); it != viewpoint_path_order_copy_.end(); ++it) {
      const ViewpointPlanner::ViewpointEntryIndex from_index = std::get<0>(viewpoint_path_copy_[*it]);
      auto next_it = it + 1;
      if (next_it == viewpoint_path_order_copy_.end()) {
        next_it = viewpoint_path_order_copy_.begin();
      }
      const ViewpointPlanner::ViewpointEntryIndex to_index = std::get<0>(viewpoint_path_copy_[*next_it]);
      const ViewpointPlanner::Motion& motion = planner_->getViewpointMotion(from_index, to_index);
      // Careful, assuming random access iterators
      for (auto it2 = motion.poses.cbegin() + 1; it2 < motion.poses.cend(); ++it2) {
        const Vector3 from_pos = (it2 - 1)->getWorldPosition();
        const Vector3 to_pos = it2->getWorldPosition();
        total_distance += (from_pos - to_pos).norm();
      }
    }

    FloatType acc_dist = 0;
    for (auto it = viewpoint_path_order_copy_.begin(); it != viewpoint_path_order_copy_.end(); ++it) {
      const ViewpointPlanner::ViewpointEntryIndex from_index = std::get<0>(viewpoint_path_copy_[*it]);
      auto next_it = it + 1;
      if (next_it == viewpoint_path_order_copy_.end()) {
        next_it = viewpoint_path_order_copy_.begin();
      }
      const ViewpointPlanner::ViewpointEntryIndex to_index = std::get<0>(viewpoint_path_copy_[*next_it]);
      const ViewpointPlanner::Motion& motion = planner_->getViewpointMotion(from_index, to_index);
      // Careful, assuming random access iterators
      for (auto it2 = motion.poses.cbegin() + 1; it2 < motion.poses.cend(); ++it2) {
        const Vector3 from_pos = (it2 - 1)->getWorldPosition();
        const Vector3 to_pos = it2->getWorldPosition();
        const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
        const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
        const FloatType local_dist = (from_pos - to_pos).norm();
        const FloatType value1 = ait::normalize<FloatType>(acc_dist, 0, total_distance);
        const FloatType value2 = ait::normalize<FloatType>(acc_dist + local_dist, 0, total_distance);
        acc_dist += local_dist;
        ait::Color3<FloatType> color1 = cmap.map(value1);
        const OGLColorData line_color1(color1.r(), color1.g(), color1.b(), 1.0f);
        ait::Color3<FloatType> color2 = cmap.map(value2);
        const OGLColorData line_color2(color2.r(), color2.g(), color2.b(), 1.0f);
        lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color1), OGLVertexDataRGBA(vertex2, line_color2));
      }
    }
  }
  viewpoint_motion_line_drawer_.upload(lines);

  lock.unlock();

  update();
}

void ViewerWidget::showSparseReconstruction(const SparseReconstruction* sparse_recon) {
    sparse_recon_ = sparse_recon;
    if (!initialized_) {
        return;
    }

    // Fill camera poses dropbox in settings panel
    std::vector<std::pair<std::string, ImageId>> pose_entries;
    for (const auto& image_entry : sparse_recon_->getImages()) {
        pose_entries.push_back(std::make_pair(image_entry.second.name(), image_entry.first));
    }
    settings_panel_->initializeImagePoses(pose_entries);

    sparce_recon_drawer_.setSparseReconstruction(sparse_recon_);

    update();
}

void ViewerWidget::refreshTree()
{
    if (octree_ != nullptr) {
        octree_drawer_.setOctree(octree_);
    }
    update();
}

void ViewerWidget::setDrawRaycast(bool draw_raycast) {
  octree_drawer_.setDrawRaycast(draw_raycast);
  update();
}

void ViewerWidget::captureRaycast() {
  Pose camera_pose = getCameraPose();
//  std::cout << "raycast pose: " << camera_pose << std::endl;
//  std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, FloatType>> raycast_results = planner_->getRaycastHitVoxels(camera_pose);
  std::cout << "BVH bounding box: " << planner_->getBvhTree().getRoot()->getBoundingBox() << std::endl;
#if WITH_CUDA
//  // Test code for Cuda raycast
//  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results_non_cuda
//    = planner_->getRaycastHitVoxels(camera_pose);
//  std::cout << "Non-cuda raycast hit " << raycast_results_non_cuda.size() << " voxels" << std::endl;
//  // End of test code
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results = planner_->getRaycastHitVoxelsCuda(camera_pose);
#else
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results = planner_->getRaycastHitVoxels(camera_pose);
#endif
  std::cout << "Raycast hit " << raycast_results.size() << " voxels" << std::endl;
  std::vector<std::pair<ViewpointPlannerData::OccupiedTreeType::IntersectionResult, FloatType>> tmp;
  tmp.reserve(raycast_results.size());
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    FloatType information = planner_->computeInformationScore(*it);
    tmp.push_back(std::make_pair(*it, information));
  }
  octree_drawer_.updateRaycastVoxels(tmp);
  update();
}

void ViewerWidget::captureRaycastWindow(const std::size_t width, const std::size_t height) {
  Pose camera_pose = getCameraPose();
//  std::cout << "raycast pose: " << camera_pose << std::endl;
#if WITH_CUDA
//  // Test code for Cuda raycast
//  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results_non_cuda
//    = planner_->getRaycastHitVoxels(camera_pose, width, height);
//  std::cout << "Non-cuda raycast hit " << raycast_results_non_cuda.size() << " voxels" << std::endl;
//  // End of test code
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results
    = planner_->getRaycastHitVoxelsCuda(camera_pose, width, height);
#else
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results
    = planner_->getRaycastHitVoxels(camera_pose, width, height);
#endif
  std::cout << "Raycast hit " << raycast_results.size() << " voxels" << std::endl;
  std::cout << "Nodes:" << std::endl;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    std::cout << "  position=" << it->node->getBoundingBox().getCenter().transpose() << std::endl;
  }
//  std::cout << "Node with weight > 0.5" << std::endl;
//  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
//    if (it->node->getObject()->weight > 0.5) {
//      std::cout << "  &node_idx=" << planner_->getBvhTree().getVoxelIndexMap().at(it->node)
//          << ", weight=" << it->node->getObject()->weight
//          << ", obs_count=" << it->node->getObject()->observation_count << std::endl;
//    }
//  }
  std::vector<std::pair<ViewpointPlannerData::OccupiedTreeType::IntersectionResult, FloatType>> tmp;
  tmp.reserve(raycast_results.size());
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    FloatType information = planner_->computeInformationScore(*it);
    tmp.push_back(std::make_pair(*it, information));
  }
  octree_drawer_.updateRaycastVoxels(tmp);
  update();
}

void ViewerWidget::captureRaycastCenter() {
  const std::size_t width = 0;
  const std::size_t height = 0;
  Pose camera_pose = getCameraPose();
//  std::cout << "raycast pose: " << camera_pose << std::endl;
#if WITH_CUDA
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results
    = planner_->getRaycastHitVoxelsCuda(camera_pose, width, height);
#else
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results
    = planner_->getRaycastHitVoxels(camera_pose, width, height);
#endif
  AIT_ASSERT(raycast_results.size() <= 1);
  std::cout << "Raycast hit " << raycast_results.size() << " voxels" << std::endl;
  std::cout << "Nodes:" << std::endl;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    std::cout << "  position=" << it->node->getBoundingBox().getCenter().transpose() << std::endl;
  }
//  std::cout << "Node with weight > 0.5" << std::endl;
//  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
//    if (it->node->getObject()->weight > 0.5) {
//      std::cout << "  &node_idx=" << planner_->getBvhTree().getVoxelIndexMap().at(it->node)
//          << ", weight=" << it->node->getObject()->weight
//          << ", obs_count=" << it->node->getObject()->observation_count << std::endl;
//    }
//  }
  std::vector<std::pair<ViewpointPlannerData::OccupiedTreeType::IntersectionResult, FloatType>> tmp;
  tmp.reserve(raycast_results.size());
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    FloatType information = planner_->computeInformationScore(*it);
    tmp.push_back(std::make_pair(*it, information));
  }
  octree_drawer_.updateRaycastVoxels(tmp);
  if (raycast_results.empty()) {
    sendClearSelectedPositionToWebSocketClients();
  }
  else {
    sendSelectedPositionToWebSocketClients(raycast_results.front().node->getBoundingBox().getCenter());
  }
  update();
}

void ViewerWidget::resetView() {
    this->camera()->setOrientation((FloatType) -M_PI / 2.0f, (FloatType) M_PI / 2.0f);
    this->showEntireScene();
    updateGL();
}

qglviewer::Vec ViewerWidget::eigenToQglviewer(const Vector3& eig_vec) const {
    return qglviewer::Vec(eig_vec(0), eig_vec(1), eig_vec(2));
}

ViewerWidget::Vector3 ViewerWidget::qglviewerToEigen(const qglviewer::Vec& qgl_vec) const {
    Vector3 eig_vec;
    eig_vec << qgl_vec.x, qgl_vec.y, qgl_vec.z;
    return eig_vec;
}

qglviewer::Quaternion ViewerWidget::eigenToQglviewer(const Quaternion& eig_quat) const {
    qglviewer::Quaternion qgl_quat(eig_quat.x(), eig_quat.y(), eig_quat.z(), eig_quat.w());
    return qgl_quat;
}

ViewerWidget::Quaternion ViewerWidget::qglviewerToEigen(const qglviewer::Quaternion& qgl_quat) const {
    Quaternion eig_quat;
    eig_quat.x() = qgl_quat[0];
    eig_quat.y() = qgl_quat[1];
    eig_quat.z() = qgl_quat[2];
    eig_quat.w() = qgl_quat[3];
    return eig_quat;
}

// Return camera pose (transformation from world to camera coordinate system)
ViewerWidget::Pose ViewerWidget::getCameraPose() const {
    Pose pose;
    pose.translation() = qglviewerToEigen(camera()->position());
    // Convert to OpenGL camera coordinate system (x is right, y is up, z is back)
    AngleAxis rotate_x_pi = AngleAxis(M_PI, Vector3::UnitX());
    pose.quaternion() = qglviewerToEigen(camera()->orientation()) * rotate_x_pi.inverse();
    return pose;
//    qglviewer::Vec translation = this->camera()->frame()->translation();
//    qglviewer::Quaternion quaternion = this->camera()->frame()->rotation();
//    quaternion.invert();
//    translation = - (quaternion * translation);
//    Matrix4x4 camera_pose = Matrix4x4::Identity();
//    for (size_t i = 0; i < 4; ++i) {
//        for (size_t j = 0; j < 4; ++j) {
//            camera_pose(i, j) = quaternion.matrix()[j * 4 + i];
//        }
//    }
//    for (size_t i = 0; i < 3; ++i) {
//        camera_pose(i, 3) = translation.v_[i];
//    }
//    return camera_pose;
}

// Set camera pose (transformation from world to camera coordinate system)
void ViewerWidget::setCameraPose(const Pose& pose) {
  camera()->setPosition(eigenToQglviewer(pose.translation()));
  // Convert to OpenGL camera coordinate system (x is right, y is up, z is back)
  AngleAxis rotate_x_pi = AngleAxis(M_PI, Vector3::UnitX());
  camera()->setOrientation(eigenToQglviewer(pose.quaternion() * rotate_x_pi));
  update();
  std::cout << "Setting camera pose to " << pose << std::endl;
//  std::cout << "get pose: " << getCameraPose() << std::endl;
}

void ViewerWidget::setOccupancyBinThreshold(double occupancy_threshold)
{
//    std::cout << "Setting occupancy threshold to " << occupancy_threshold << std::endl;
    octree_drawer_.setOccupancyBinThreshold(occupancy_threshold);
    update();
}

void ViewerWidget::setColorFlags(uint32_t color_flags)
{
//    std::cout << "Setting color flags to " << color_flags << std::endl;
    octree_drawer_.setColorFlags(color_flags);
    update();
}

void ViewerWidget::setDrawFreeVoxels(bool draw_free_voxels)
{
    octree_drawer_.setDrawFreeVoxels(draw_free_voxels);
    update();
}

void ViewerWidget::setDrawAxes(bool draw_axes)
{
  axes_drawer_.setDrawLines(draw_axes);
  update();
}

void ViewerWidget::setVoxelAlpha(double voxel_alpha) {
//    std::cout << "Setting voxel alpha to " << voxel_alpha << std::endl;
    octree_drawer_.setAlphaOccupied(voxel_alpha);
    update();
}

void ViewerWidget::setDrawSingleBin(bool draw_single_bin) {
    octree_drawer_.setDrawSingleBin(draw_single_bin);
    update();
}

void ViewerWidget::setDrawOctree(bool draw_octree) {
  octree_drawer_.setDrawOctree(draw_octree);
  update();
}

void ViewerWidget::setDrawCameras(bool draw_cameras) {
    sparce_recon_drawer_.setDrawCameras(draw_cameras);
    update();
}

void ViewerWidget::setDrawViewpointGraph(bool draw_viewpoint_graph) {
  viewpoint_graph_drawer_.setDrawCameras(draw_viewpoint_graph);
  update();
}

void ViewerWidget::setDrawViewpointMotions(bool draw_viewpoint_motions) {
  viewpoint_motion_line_drawer_.setDrawLines(draw_viewpoint_motions);
  update();
}

void ViewerWidget::setDrawViewpointPath(bool draw_viewpoint_path) {
  viewpoint_path_drawer_.setDrawCameras(draw_viewpoint_path);
  update();
}

void ViewerWidget::setDrawSparsePoints(bool draw_sparse_points) {
  sparce_recon_drawer_.setDrawSparsePoints(draw_sparse_points);
  update();
}

void ViewerWidget::setDrawDensePoints(bool draw_dense_points) {
  dense_points_drawer_.setDrawPoints(draw_dense_points);
  update();
}

void ViewerWidget::setDrawPoissonMesh(bool draw_poisson_mesh) {
  poisson_mesh_drawer_.setDrawTriangles(draw_poisson_mesh);
  update();
}

void ViewerWidget::setDrawRegionOfInterest(bool draw_region_of_interest) {
  region_of_interest_drawer_.setDrawLines(draw_region_of_interest);
  update();
}

void ViewerWidget::setDrawBvhBbox(bool draw_bvh_bbox) {
  bvh_bbox_drawer_.setDrawLines(draw_bvh_bbox);
  update();
}

void ViewerWidget::setUseDroneCamera(bool use_drone_camera) {
  if (use_drone_camera) {
    const reconstruction::PinholeCameraColmap& pinhole_camera = sparse_recon_->getCameras().cbegin()->second;
    double fy = pinhole_camera.getFocalLengthY();
    double v_fov = 2 * std::atan(pinhole_camera.height() / (2 * fy));
    camera()->setFieldOfView(v_fov);
    aspect_ratio_ = pinhole_camera.width() / static_cast<double>(pinhole_camera.height());
    updateGeometry();
    std::cout << "Setting camera FOV to " << (v_fov * 180 / M_PI) << " degrees" << std::endl;
    std::cout << "Resized window to " << width() << " x " << height() << std::endl;
  }
  else {
    double v_fov = M_PI / 4;
    camera()->setFieldOfView(v_fov);
    std::cout << "Setting camera FOV to " << (v_fov * 180 / M_PI) << std::endl;
  }
  update();
}

void ViewerWidget::setImagePoseIndex(ImageId image_id) {
  const ImageColmap& image = sparse_recon_->getImages().at(image_id);
  setCameraPose(image.pose());
  std::cout << "Selected image with id " << image_id << std::endl;
  std::cout << "  Pose=" << image.pose().getWorldPosition().transpose() << std::endl;
  std::cout << "  GPS=" << getGpsFromPose(image.pose()) << std::endl;
  update();
}

void ViewerWidget::setViewpointGraphSelectionIndex(const std::size_t index) {
  std::cout << "Selected viewpoint " << index << std::endl;
  std::unique_lock<std::mutex> lock(mutex_);
  const Pose& pose = std::get<1>(viewpoint_graph_copy_[index]);
  std::cout << "  Pose=" << pose.getWorldPosition().transpose() << std::endl;
  std::cout << "  GPS=" << getGpsFromPose(pose) << std::endl;
  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(viewpoint_graph_copy_[index]);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
  const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
  const FloatType total_information = viewpoint_entry.total_information;
  const FloatType new_information = planner_->computeNewInformation(viewpoint_path_branch_index_, viewpoint_index);
  std::cout << "  Total information=" << total_information << std::endl;
  std::cout << "  New information=" << new_information << std::endl;
  if (planner_panel_->isUpdateCameraOnSelectionChecked()) {
    setCameraPose(viewpoint_entry.viewpoint.pose());
  }
  octree_drawer_.updateRaycastVoxels(viewpoint_entry.voxel_set);
  lock.unlock();
  showViewpointGraph(index);
//  // Set previous selection for combo box
  planner_panel_->setViewpointGraphSelection(index);
}

void ViewerWidget::setViewpointPathBranchSelectionIndex(std::size_t index) {
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_path_branch_index_ = index;
  std::cout << "Selected viewpoint path " << index << std::endl;
  const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[index];
  for (std::size_t i = 0; i < viewpoint_path.entries.size(); ++i) {
    std::cout << "  [" << i << "] information=" << viewpoint_path.entries[i].local_information
        << ", motion distance=" << viewpoint_path.entries[i].local_motion_distance
        << ", objective=" << viewpoint_path.entries[i].local_objective << std::endl;
  }
  std::cout << "  accumulated information=" << viewpoint_path.acc_information << std::endl;
  std::cout << "  accumulated motion distance=" << viewpoint_path.acc_motion_distance << std::endl;
  std::cout << "  accumulated objective=" << viewpoint_path.acc_objective << std::endl;
  lock.unlock();
  planner_thread_.updateViewpoints();
  // Remember selected entry before updating combo box
  const std::size_t path_selection_index = planner_panel_->getViewpointPathSelection();
  showViewpointPath();
  // Set previous selection for combo box
  planner_panel_->setViewpointPathBranchSelection(index);
  // Set previous selection for combo box
  planner_panel_->setViewpointPathSelectionByItemIndex(path_selection_index);
  // Show triangulated voxels
  octree_drawer_.updateRaycastVoxels(viewpoint_path.observed_voxel_set);
}

void ViewerWidget::setViewpointPathSelectionIndex(std::size_t index) {
  const bool verbose = true;

  std::unique_lock<std::mutex> lock(mutex_);
  const Pose& pose = std::get<1>(viewpoint_path_copy_[index]);
  std::cout << "  Pose=" << pose.getWorldPosition().transpose() << std::endl;
  std::cout << "  GPS=" << getGpsFromPose(pose) << std::endl;
//  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(viewpoint_path_copy_[index]);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();

  // Compute total and incremental voxel set and information of selected viewpoint
  const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index_];
  const ViewpointPlanner::ViewpointPathComputationData& comp_data = planner_->getViewpointPathsComputationData()[viewpoint_path_branch_index_];
  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[index].viewpoint_index;
  const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
  ViewpointPlanner::VoxelWithInformationSet total_voxel_set = viewpoint_entry.voxel_set;
  // TODO: Broken. Only for triangulation mode,
  if (!comp_data.triangulated_voxel_to_path_entries_map.empty()) {
    for (const auto& entry : comp_data.triangulated_voxel_to_path_entries_map) {
      auto it1 = std::find_if(total_voxel_set.begin(), total_voxel_set.end(),
          [&](const ViewpointPlanner::VoxelWithInformation& vi) {
        return vi.voxel == entry.first;
      });
      if (it1 != total_voxel_set.end()) {
        auto it2 = std::find_if(entry.second.begin(), entry.second.end(),
            [&](const std::pair<ViewpointPlanner::ViewpointEntryIndex, ViewpointPlanner::ViewpointEntryIndex>& p) {
          return p.first == viewpoint_index || p.second == viewpoint_index;
        });
        if (it2 == entry.second.end()) {
          total_voxel_set.erase(it1);
        }
      }
    }
  }
  ViewpointPlanner::VoxelWithInformationSet incremental_voxel_set = total_voxel_set;
  ViewpointPlanner::VoxelWithInformationSet accumulated_voxel_set = total_voxel_set;
  for (std::size_t i = 0; i < index; ++i) {
    ViewpointPlanner::ViewpointEntryIndex other_viewpoint_index = viewpoint_path.entries[i].viewpoint_index;
    const ViewpointPlanner::VoxelWithInformationSet& other_voxel_set = planner_->getViewpointEntries()[other_viewpoint_index].voxel_set;
    for (const ViewpointPlanner::VoxelWithInformation& voxel_with_information : other_voxel_set) {
      incremental_voxel_set.erase(voxel_with_information);
    }
    accumulated_voxel_set.insert(other_voxel_set.cbegin(), other_voxel_set.cend());
  }

  if (verbose) {
    // Print some information on the selected viewpoint
    std::cout << "Selected viewpoint " << viewpoint_index << std::endl;
    std::cout << "  total_voxel_set.size()=" << total_voxel_set.size() << std::endl;
    std::cout << "  incremental_voxel_set.size()=" << incremental_voxel_set.size() << std::endl;
    const std::size_t total_informative_voxel_count = std::count_if(total_voxel_set.cbegin(), total_voxel_set.cend(),
        [](const ViewpointPlanner::VoxelWithInformation& voxel) {
          return voxel.information > 0;
    });
    std::cout << "  total informative voxel count=" << total_informative_voxel_count << std::endl;
    const std::size_t incremental_informative_voxel_count = std::count_if(incremental_voxel_set.cbegin(), incremental_voxel_set.cend(),
        [](const ViewpointPlanner::VoxelWithInformation& voxel) {
          return voxel.information > 0;
    });
    std::cout << "  incremental informative voxel count=" << incremental_informative_voxel_count << std::endl;
    const FloatType total_voxel_information = std::accumulate(total_voxel_set.cbegin(), total_voxel_set.cend(),
        FloatType { 0 }, [](const FloatType& value, const ViewpointPlanner::VoxelWithInformation& voxel) {
          return value + voxel.information;
    });
    std::cout << "  total voxel information=" << total_voxel_information << std::endl;
    const FloatType incremental_voxel_information = std::accumulate(incremental_voxel_set.cbegin(), incremental_voxel_set.cend(),
        FloatType { 0 }, [](const FloatType& value, const ViewpointPlanner::VoxelWithInformation& voxel) {
          return value + voxel.information;
    });
    std::cout << "  incremental voxel information=" << incremental_voxel_information << std::endl;
  }

  if (planner_panel_->isUpdateCameraOnSelectionChecked()) {
    setCameraPose(viewpoint_entry.viewpoint.pose());
  }
  if (planner_panel_->isShowIncrementalVoxelSetChecked()) {
    octree_drawer_.updateRaycastVoxels(incremental_voxel_set);
  }
  else if (planner_panel_->isShowAccumulativeVoxelSetChecked()) {
    octree_drawer_.updateRaycastVoxels(accumulated_voxel_set);
  }
  else {
    octree_drawer_.updateRaycastVoxels(total_voxel_set);
  }
  lock.unlock();
  // Remember selected entry before updating combo box
  const std::size_t branch_selection_index = planner_panel_->getViewpointPathBranchSelection();
  showViewpointPath(index);
  // Set previous selection for combo box
  planner_panel_->setViewpointPathSelection(index);
  planner_panel_->setViewpointPathBranchSelection(branch_selection_index);
}

void ViewerWidget::setMinOccupancy(double min_occupancy) {
  octree_drawer_.setMinOccupancy(min_occupancy);
  update();
}

void ViewerWidget::setMaxOccupancy(double max_occupancy) {
  octree_drawer_.setMaxOccupancy(max_occupancy);
  update();
}

void ViewerWidget::setMinObservations(uint32_t min_observations) {
  octree_drawer_.setMinObservations(min_observations);
  update();
}

void ViewerWidget::setMaxObservations(uint32_t max_observations) {
  octree_drawer_.setMaxObservations(max_observations);
  update();
}

void ViewerWidget::setMinVoxelSize(double min_voxel_size) {
  octree_drawer_.setMinVoxelSize(min_voxel_size);
  update();
}

void ViewerWidget::setMaxVoxelSize(double max_voxel_size) {
  octree_drawer_.setMaxVoxelSize(max_voxel_size);
  update();
}

void ViewerWidget::setMinWeight(double min_weight) {
  octree_drawer_.setMinWeight(min_weight);
  update();
}

void ViewerWidget::setMaxWeight(double max_weight) {
  octree_drawer_.setMaxWeight(max_weight);
  update();
}

void ViewerWidget::setMinInformation(double min_information) {
  octree_drawer_.setMinInformation(min_information);
  update();
}

void ViewerWidget::setMaxInformation(double max_information) {
  octree_drawer_.setMaxInformation(max_information);
  update();
}

void ViewerWidget::setRenderTreeDepth(std::size_t render_tree_depth)
{
//    std::cout << "Setting render tree depth to " << render_tree_depth << std::endl;
    octree_drawer_.setRenderTreeDepth(render_tree_depth);
    update();
}

void ViewerWidget::setRenderObservationThreshold(std::size_t render_observation_threshold)
{
//    std::cout << "Setting render observation threshold to " << observation_threshold << std::endl;
    octree_drawer_.setRenderObservationThreshold(render_observation_threshold);
    update();
}

void ViewerWidget::setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max)
{
//    qglviewer::Vec min_vec(-50, -50, -20);
//    qglviewer::Vec max_vec(50, 50, 50);
//    QGLViewer::setSceneBoundingBox(min_vec, max_vec);
    QGLViewer::setSceneBoundingBox(min, max);
}

void ViewerWidget::draw() {
    glEnable(GL_DEPTH_TEST);
//  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  glEnable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
//    glEnable(GL_CULL_FACE);
//    glCullFace(GL_BACK);

//    camera()->fitBoundingBox(qglviewer::Vec(-10, -10, -10), qglviewer::Vec(10, 10, 10));
//    camera()->setSceneCenter(qglviewer::Vec(0, 0, 0));
  QMatrix4x4 pvm_matrix;
  camera()->getModelViewProjectionMatrix(pvm_matrix.data());
  QMatrix4x4 view_matrix;
  camera()->getModelViewMatrix(view_matrix.data());
  QMatrix4x4 model_matrix; // Identity
  model_matrix.setToIdentity();

  // draw drawable objects:
  octree_drawer_.draw(pvm_matrix, view_matrix, model_matrix);
  sparce_recon_drawer_.draw(pvm_matrix, width(), height());
  dense_points_drawer_.draw(pvm_matrix, dense_points_size_);
  poisson_mesh_drawer_.draw(pvm_matrix);
  region_of_interest_drawer_.draw(pvm_matrix, width(), height(), bbox_line_width_);
  bvh_bbox_drawer_.draw(pvm_matrix, width(), height(), bbox_line_width_);
  // Draw path before graph so that the graph is hidden
  viewpoint_path_drawer_.draw(pvm_matrix, width(), height());
  viewpoint_motion_line_drawer_.draw(pvm_matrix, width(), height(), viewpoint_motion_line_width_);
  viewpoint_graph_drawer_.draw(pvm_matrix, width(), height());

  // Draw coordinate axes
  if (axes_drawer_.getDrawLines()) {
    glDisable(GL_DEPTH_TEST);
    const std::size_t axes_viewport_width = width() / 8;
    const std::size_t axes_viewport_height = height() / 8;
    glViewport(0, 0, axes_viewport_width, axes_viewport_height);
    QMatrix4x4 mv_matrix;
    camera()->getModelViewMatrix(mv_matrix.data());
    mv_matrix.setColumn(3, QVector4D(0, 0, -5, 1));
    camera()->getProjectionMatrix(pvm_matrix.data());
    pvm_matrix.setToIdentity();
    pvm_matrix.ortho(-1.5f, 1.5f, -1.5f, 1.5f, 0.1f, 10.0f);
    pvm_matrix *= mv_matrix;
    axes_drawer_.draw(pvm_matrix, width(), height(), 40.0f);
    glViewport(0, 0, width(), height());
    glEnable(GL_DEPTH_TEST);
  }
}

void ViewerWidget::drawWithNames() {}

void ViewerWidget::postDraw() {}

void ViewerWidget::postSelection(const QPoint&) {}

void ViewerWidget::wheelEvent(QWheelEvent* event) {
  if (event->modifiers() & Qt::ShiftModifier) {
    if (event->delta() != 0) {
      std::cout << (1.0 - event->delta() / 100.0 * kZFarSpeed) << std::endl;
      std::cout << event->delta() << std::endl;
      if (event->modifiers() & Qt::ControlModifier) {
        custom_camera_.setZFar(custom_camera_.zFar() * (1.0 - event->delta() / 100.0 * kZFarSpeed));
        custom_camera_.setZFar(ait::clamp(custom_camera_.zFar(), kZFarMin, kZFarMax));
      }
      else {
        custom_camera_.setZNear(custom_camera_.zNear() * (1.0 + event->delta() / 100.0 * kZNearSpeed));
        custom_camera_.setZNear(ait::clamp(custom_camera_.zNear(), kZNearMin, kZNearMax));
      }
      std::cout << "zNear: " << camera()->zNear() << ", zFar: " << camera()->zFar() << std::endl;
      std::cout << "scene center: " << camera()->sceneCenter() << std::endl;
      std::cout << "pivot point: " << camera()->pivotPoint() << std::endl;
    }
    event->accept();
    updateGL();
  }
  else if (event->modifiers() & Qt::ControlModifier) {
    sparce_recon_drawer_.changePointSize(event->delta());
    event->accept();
    updateGL();
  } else if (event->modifiers() & Qt::AltModifier) {
    sparce_recon_drawer_.changeCameraSize(event->delta());
    viewpoint_graph_drawer_.changeCameraSize(event->delta());
    viewpoint_path_drawer_.changeCameraSize(event->delta());
    event->accept();
    updateGL();
  } else {
//      ChangeFocusDistance(event->delta());
    QGLViewer::wheelEvent(event);
  }
}

void ViewerWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_P) {
    const std::string screenshot_dir = "screenshots/";
    const QDir currentDir(".");
    if (!currentDir.exists(QString::fromStdString(screenshot_dir))) {
      if (!currentDir.mkdir(QString::fromStdString(screenshot_dir))) {
        std::cout << "ERROR: Unable to create screenshot folder" << std::endl;
        return;
      }
    }
    const std::string format_ext = ".png";
    const std::string date_string = QDateTime::currentDateTime().toString("dd.MM.yy_hh.mm.ss").toStdString();
    std::string screenshot_filename = screenshot_dir + std::string("Quad3DR_screenshort_") + date_string + format_ext;
    if (event->modifiers() == Qt::ShiftModifier) {
      screenshot_filename = QFileDialog::getSaveFileName(this, tr("Save screenshot"),
          QString::fromStdString(screenshot_filename),
          tr("PNG Image (*.png);;JPEG Image (*.jpeg|.jpg);;All Files (*.*)")).toStdString();
    }
    if (!screenshot_filename.empty()) {
      saveScreenshot(screenshot_filename);
      std::cout << "Saved screenshot" << std::endl;
    }
  }
  else if (event->key() == Qt::Key_C) {
    const Pose pose = getCameraPose();
    std::cout << "Camera pose = " << pose << std::endl;
    std::cout << "Axis: " << std::endl;
    std::cout << " x = " << pose.quaternion().toRotationMatrix().col(0).transpose() << std::endl;
    std::cout << " y = " << pose.quaternion().toRotationMatrix().col(1).transpose() << std::endl;
    std::cout << " z = " << pose.quaternion().toRotationMatrix().col(2).transpose() << std::endl;
    std::cout << "GPS = " << getGpsFromPose(pose) << std::endl;
  }
  else if (event->key() == Qt::Key_T) {
    captureRaycast();
  }
  else if (event->key() == Qt::Key_R) {
    captureRaycastCenter();
    captureRaycastWindow(15, 15);
  }
  else {
    QGLViewer::keyPressEvent(event);
  }
}

ViewerWidget::GpsCoordinateType ViewerWidget::getGpsFromPose(const Pose& pose) const {
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = ait::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = planner_->getReconstruction()->sfmGpsTransformation().gps_reference;
  std::cout << "GPS reference: " << gps_reference << std::endl;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);
  const Vector3 enu = pose.getWorldPosition();
  GpsCoordinateType gps = gps_converter.convertEnuToGps(enu.cast<GpsFloatType>());
  return gps;
}

void ViewerWidget::saveScreenshot(const std::string& filename) {
  const bool with_alpha = true;
  QImage frame = this->grabFrameBuffer(with_alpha);
  frame.save(QString::fromStdString(filename), nullptr, kScreenshotQuality);
}

void ViewerWidget::pausePlannerThread() {
  if (planner_thread_.isRunning()) {
    planner_panel_->setAllComputationButtonsEnabled(false);
    planner_thread_.signalPause();
  }
//  else {
//    planner_panel_->setPauseContinueViewpointGraphEnabled(true);
//    planner_panel_->setPauseContinueViewpointGraphText("Continue");
//    planner_panel_->setPauseContinueViewpointPathEnabled(true);
//    planner_panel_->setPauseContinueViewpointPathText("Continue");
//  }
}

void ViewerWidget::continuePlannerThread() {
  if (planner_thread_.operation() != ViewpointPlannerThread::Operation::NOP) {
    planner_panel_->setAllComputationButtonsEnabled(false);
    planner_panel_->setAllResetButtonsEnabled(false);
  }
  if (planner_thread_.operation() == ViewpointPlannerThread::Operation::VIEWPOINT_GRAPH) {
    planner_panel_->setPauseContinueViewpointGraphEnabled(true);
    planner_panel_->setPauseContinueViewpointGraphText("Pause");
  }
  else if (planner_thread_.operation() == ViewpointPlannerThread::Operation::VIEWPOINT_MOTIONS) {
    planner_panel_->setPauseContinueViewpointMotionsEnabled(true);
    planner_panel_->setPauseContinueViewpointMotionsText("Pause");
  }
  else if (planner_thread_.operation() == ViewpointPlannerThread::Operation::VIEWPOINT_PATH) {
    planner_panel_->setPauseContinueViewpointPathEnabled(true);
    planner_panel_->setPauseContinueViewpointPathText("Pause");
  }
  else if (planner_thread_.operation() == ViewpointPlannerThread::Operation::VIEWPOINT_PATH_TSP) {
    planner_panel_->setSolveViewpointTSPEnabled(false);
    planner_panel_->setPauseContinueViewpointPathText("Solving");
  }
  planner_thread_.signalContinue();
}

void ViewerWidget::onPlannerThreadPaused() {
  planner_panel_->setAllComputationButtonsEnabled(true);
  planner_panel_->setAllComputationButtonsText("Continue");
  planner_panel_->setSolveViewpointTSPText("Solve");
  planner_panel_->setAllResetButtonsEnabled(true);
}

void ViewerWidget::pauseContinueViewpointGraph() {
  pauseContinueOperation(ViewpointPlannerThread::Operation::VIEWPOINT_GRAPH);
}

void ViewerWidget::pauseContinueViewpointMotions() {
  pauseContinueOperation(ViewpointPlannerThread::Operation::VIEWPOINT_MOTIONS);
}

void ViewerWidget::pauseContinueViewpointPath() {
  pauseContinueOperation(ViewpointPlannerThread::Operation::VIEWPOINT_PATH);
}

void ViewerWidget::solveViewpointTSP() {
  pauseContinueOperation(ViewpointPlannerThread::Operation::VIEWPOINT_PATH_TSP);
}

void ViewerWidget::pauseContinueOperation(ViewpointPlannerThread::Operation operation) {
  if (planner_thread_.operation() != operation) {
    planner_thread_.setOperation(operation);
  }
  if (planner_thread_.isPaused()) {
    continuePlannerThread();
  }
  else {
    pausePlannerThread();
  }
}

void ViewerWidget::resetViewpoints() {
  // TODO: Call asynchronously?
  planner_->reset();
  planner_thread_.updateViewpoints();
  showViewpointGraph();
  showViewpointPath();
}

void ViewerWidget::resetViewpointMotions() {
  // TODO: Call asynchronously?
  planner_->resetViewpointMotions();
}

void ViewerWidget::resetViewpointPath() {
  // TODO: Call asynchronously?
  planner_->resetViewpointPaths();
  planner_thread_.updateViewpoints();
  showViewpointGraph();
  showViewpointPath();
}

void ViewerWidget::onSaveViewpointGraph(const std::string& filename) {
  planner_->saveViewpointGraph(filename);
}

void ViewerWidget::onLoadViewpointGraph(const std::string& filename) {
  planner_->loadViewpointGraph(filename);
  planner_thread_.updateViewpoints();
  showViewpointGraph();
  showViewpointPath();
}

void ViewerWidget::onSaveViewpointPath(const std::string& filename) {
  planner_->saveViewpointPath(filename);
}

void ViewerWidget::onLoadViewpointPath(const std::string& filename) {
  planner_->loadViewpointPath(filename);
  planner_thread_.updateViewpoints();
  showViewpointPath();
}

void ViewerWidget::onExportViewpointPathAsJson(const std::string& filename) {
  if (viewpoint_path_branch_index_ != (std::size_t)-1 && viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    std::cout << "Exporting viewpoint path " << viewpoint_path_branch_index_ << std::endl;
    planner_->exportViewpointPathAsJson(filename, planner_->getViewpointPaths()[viewpoint_path_branch_index_]);
  }
  else {
    std::cout << "Exporting best viewpoint path" << std::endl;
    planner_->exportViewpointPathAsJson(filename, planner_->getBestViewpointPath());
  }
  std::cout << "Done" << std::endl;
}

void ViewerWidget::signalViewpointsChanged() {
  emit viewpointsChanged();
}

void ViewerWidget::signalPlannerThreadPaused() {
  emit plannerThreadPaused();
}

// TODO: Remove
void ViewerWidget::setUseFixedColors(bool use_fixed_colors) {
  showViewpointGraph();
  showViewpointPath();
}

void ViewerWidget::setAlphaParameter(double alpha) {
  planner_thread_.setAlpha(alpha);
}

void ViewerWidget::setBetaParameter(double beta) {
  planner_thread_.setBeta(beta);
}

void ViewerWidget::setMinInformationFilter(double min_information_filter) {
  min_information_filter_ = (FloatType)min_information_filter;
  showViewpointGraph();
  showViewpointPath();
}

void ViewerWidget::setViewpointPathLineWidth(double line_width) {
  viewpoint_motion_line_width_ = (FloatType)line_width;
  update();
}

void ViewerWidget::setViewpointColorMode(std::size_t color_mode) {
  viewpoint_color_mode_ = static_cast<ViewpointColorMode>(color_mode);
  showViewpointGraph(planner_panel_->getViewpointGraphSelection());
}

void ViewerWidget::setViewpointGraphComponent(int component) {
  viewpoint_selected_component_ = component;
  std::cout << "Selected component: " << component << std::endl;
  showViewpointGraph(planner_panel_->getViewpointGraphSelection());
  planner_panel_->setViewpointComponentSelection(viewpoint_selected_component_);
}

void ViewerWidget::updateViewpoints() {
  showViewpointGraph();
  showViewpointPath();
  sendViewpointPathToWebSocketClients();
}

void ViewerWidget::sendViewpointPathToWebSocketClients() {
  if (web_socket_server_ == nullptr) {
    return;
  }
  const ViewpointPlanner::ViewpointPath* viewpoint_path;
  if (viewpoint_path_branch_index_ != (std::size_t)-1 && viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    std::cout << "Exporting viewpoint path " << viewpoint_path_branch_index_ << std::endl;
    viewpoint_path = &planner_->getViewpointPaths()[viewpoint_path_branch_index_];
  }
  else {
    std::cout << "Exporting best viewpoint path" << std::endl;
    viewpoint_path = &planner_->getBestViewpointPath();
  }
  std::cout << "Done" << std::endl;
  rapidjson::Document json_message;
  json_message.SetObject();
  auto& allocator = json_message.GetAllocator();
  json_message.AddMember("type", "viewpoint_path", allocator);
  const rapidjson::Document json_viewpoint_path = planner_->getViewpointPathAsJson(*viewpoint_path);
  rapidjson::Value json_path;
  json_path.CopyFrom(json_viewpoint_path, allocator);
  json_message.AddMember("payload", json_path, allocator);
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  json_message.Accept(writer);
  web_socket_server_->sendTextMessage(buffer.GetString());
}

void ViewerWidget::sendClearSelectedPositionToWebSocketClients() {
  if (web_socket_server_ == nullptr) {
    return;
  }
  rapidjson::Document json_message;
  json_message.SetObject();
  auto& allocator = json_message.GetAllocator();
  json_message.AddMember("type", "clear_selected_position", allocator);
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  json_message.Accept(writer);
  web_socket_server_->sendTextMessage(buffer.GetString());
}

void ViewerWidget::sendSelectedPositionToWebSocketClients(const ViewpointPlanner::Vector3& position) {
  if (web_socket_server_ == nullptr) {
    return;
  }
  ViewpointPlanner::GpsCoordinateType gps = planner_->convertPositionToGps(position);
  rapidjson::Document json_message;
  json_message.SetObject();
  auto& allocator = json_message.GetAllocator();
  json_message.AddMember("type", "selected_position", allocator);
  rapidjson::Value json_enu(rapidjson::kObjectType);
  json_enu.AddMember("x", position(0), allocator);
  json_enu.AddMember("y", position(1), allocator);
  json_enu.AddMember("z", position(2), allocator);
  rapidjson::Value json_gps(rapidjson::kObjectType);
  json_gps.AddMember("latitude", gps.latitude(), allocator);
  json_gps.AddMember("longitude", gps.longitude(), allocator);
  json_gps.AddMember("altitude", gps.altitude(), allocator);
  rapidjson::Value selected_position(rapidjson::kObjectType);
  selected_position.AddMember("enu", json_enu, allocator);
  selected_position.AddMember("gps", json_gps, allocator);
  json_message.AddMember("payload", selected_position, allocator);
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  json_message.Accept(writer);
  web_socket_server_->sendTextMessage(buffer.GetString());
}

ViewpointPlannerThread::ViewpointPlannerThread(ViewpointPlanner* planner, ViewerWidget* viewer_widget)
: ait::PausableThread(), planner_(planner), viewer_widget_(viewer_widget),
  operation_(Operation::NOP),
  alpha_(0), beta_(0), viewpoint_path_branch_index_(0) {
  setPausedCallback([viewer_widget]() {
    viewer_widget->signalPlannerThreadPaused();
  });
}

ViewpointPlannerThread::Result ViewpointPlannerThread::runIteration() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (operation_ == VIEWPOINT_GRAPH) {
    std::cout << "Running iterations of generateNextViewpoint()" << std::endl;
    for (std::size_t i = 0; i < 10; ++i) {
      bool result = planner_->generateNextViewpointEntry();
      std::cout << "Result[" << i << "] -> " << result << std::endl;
    }
    updateViewpointsInternal();
    viewer_widget_->signalViewpointsChanged();
    return Result::CONTINUE;
  }
  else if (operation_ == VIEWPOINT_MOTIONS) {
    planner_->computeViewpointMotions();
    return Result::PAUSE;
  }
  else if (operation_ == VIEWPOINT_PATH) {
    std::cout << "Running iterations of findNextViewpointPathEntry()" << std::endl;
    // TODO: set parameter from widget
    bool result = planner_->findNextViewpointPathEntries(alpha_, beta_);
    std::cout << "Result -> " << result << std::endl;
    updateViewpointsInternal();
    viewer_widget_->signalViewpointsChanged();
    return Result::CONTINUE;
  }
  else if (operation_ == VIEWPOINT_PATH_TSP) {
    std::cout << "Solving TSP problem for viewpoint paths" << std::endl;
    planner_->computeViewpointTour();
    std::cout << "Done" << std::endl;
    updateViewpointsInternal();
    viewer_widget_->signalViewpointsChanged();
    return Result::PAUSE;
  }
  else if (operation_ == VIEWPOINT_UPDATE) {
    std::cout << "Updating display of viewpoints" << std::endl;
    updateViewpointsInternal();
    viewer_widget_->signalViewpointsChanged();
    return Result::PAUSE;
  }
  else {
    std::cout << "Giving back stop result" << std::endl;
    return Result::STOP;
  }
}

void ViewpointPlannerThread::updateViewpoints() {
//  AIT_ASSERT(!this->isRunning() || this->isPaused());
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock) {
    updateViewpointsInternal();
  }
}

void ViewpointPlannerThread::updateViewpointsInternal() {
  std::lock_guard<std::mutex> lock(viewer_widget_->mutex_);
  // Copy viewpoint graph and path to local copy for visualization
  viewer_widget_->viewpoint_graph_copy_.clear();
  for (auto it = planner_->getViewpointGraph().begin(); it != planner_->getViewpointGraph().end(); ++it) {
    const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = it.node();
    const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
    viewer_widget_->viewpoint_graph_copy_.push_back(std::make_tuple(
        viewpoint_index, viewpoint_entry.viewpoint.pose(), viewpoint_entry.total_information));
  }
  viewer_widget_->viewpoint_path_copy_.clear();
  const std::size_t path_index = viewer_widget_->viewpoint_path_branch_index_;
  for (const ViewpointPlanner::ViewpointPathEntry& path_entry : planner_->getViewpointPaths()[path_index].entries) {
    const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[path_entry.viewpoint_index];
    viewer_widget_->viewpoint_path_copy_.push_back(std::make_tuple(
        path_entry.viewpoint_index, viewpoint_entry.viewpoint.pose(), path_entry.local_information));
  }
  viewer_widget_->viewpoint_path_order_copy_ = planner_->getViewpointPaths()[path_index].order;
}
