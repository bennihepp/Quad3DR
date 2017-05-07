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
#include <QInputDialog>
#include <bh/qt/utils.h>
#include <bh/utilities.h>
#include <bh/eigen_utils.h>
#include <bh/pose.h>
#include <bh/color.h>
#include <bh/filesystem.h>

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
      selected_viewpoint_graph_entry_index_((size_t)-1),
      selected_viewpoint_path_branch_index_((size_t)-1),
      selected_viewpoint_path_entry_index_((size_t)-1),
      camera_pose_selection_valid_(false),
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
      viewpoint_path_branch_index_(0),
      raycast_mode_(RaycastMode::DEFAULT) {
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
    for (const auto& entry : rendering::VoxelDrawer::getAvailableColorFlags()) {
      color_flags_uint.push_back(std::make_pair(entry.first, static_cast<uint32_t>(entry.second)));
    }
  BH_ASSERT(!color_flags_uint.empty());
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
    connect(this, SIGNAL(raycastFinished()), this, SLOT(onRaycastFinished()));
    connect(this, SIGNAL(makeViewpointMotionsSparseMatchableFinished()), this, SLOT(onMakeViewpointMotionsSparseMatchableFinished()));

    std::vector<std::pair<std::string, std::size_t>> color_mode_size_t;
    for (const auto& entry : getAvailableViewpointColorModes()) {
      color_mode_size_t.push_back(std::make_pair(entry.first, static_cast<std::size_t>(entry.second)));
    }
    BH_ASSERT(!color_mode_size_t.empty());
    planner_panel_->initializeViewpointColorMode(color_mode_size_t);
    planner_panel_->selectViewpointColorMode(color_mode_size_t[0].second);
    planner_panel_->setAlphaParameter((double)planner_->getViewpointPathTimeConstraint());

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
      ("Information", ViewpointColorMode::Information)
      ("Indexed", ViewpointColorMode::Indexed);
  return modes;
}

void ViewerWidget::init() {
  setCamera(&custom_camera_);

  initialized_ = true;

  //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::AltModifier);
  //    setHandlerKeyboardModifiers(QGLViewer::FRAME, Qt::NoModifier);
  //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::ControlModifier);
//  setMouseTracking(true);

  // Restore previous viewer state.
  restoreStateFromFile();
  std::cout << "QGLViewer.stateFilename: " <<stateFileName().toStdString() << std::endl;

  // Make camera the default manipulated frame.
  setManipulatedFrame(camera()->frame());
  // invert mousewheel (more like Blender)
  camera()->frame()->setWheelSensitivity(-0.5);
  camera()->frame()->setSpinningSensitivity(std::numeric_limits<qreal>::max());

  // TODO
  camera()->setSceneCenter(qglviewer::Vec(0, 0, 0));
  camera()->setSceneRadius(10);

  camera()->setPosition(qglviewer::Vec(0, 0, 100));
  camera()->lookAt(qglviewer::Vec(0, 0, 0));

  setUseDroneCamera(settings_panel_->getUseDroneCamera());

  // background color defaults to white
  this->setBackgroundColor(QColor(225, 225, 225));
  this->qglClearColor(this->backgroundColor());

  initAxesDrawer();

  std::cout << "zNear: " << camera()->zNear() << ", zFar: " << camera()->zFar() << std::endl;
  // TODO: Hack, make this dependent on the scene
//  camera()->setZNearCoefficient(z_near_coefficient_);
//  camera()->setZClippingCoefficient(1);
  sparse_recon_drawer_.init();
  dense_points_drawer_.init();
  dense_points_drawer_.setDrawPoints(false);
  poisson_mesh_drawer_.init();
  poisson_mesh_drawer_.setDrawTriangles(false);
  poisson_mesh_normal_drawer_.init();
  poisson_mesh_normal_drawer_.setDrawLines(false);
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
  showRegionOfInterest();
  showBvhBbox(planner_->getBvhBbox());
}

void ViewerWidget::initAxesDrawer() {
  using rendering::OGLLineData;
  using rendering::OGLColorData;
  using rendering::OGLVertexData;
  using rendering::OGLVertexDataRGBA;
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

    bh::Timer timer;
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

//    setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));
    const ViewpointPlanner::BoundingBoxType bbox = planner_->getBvhBbox() * 2;
    setSceneBoundingBox(eigenToQglviewer(bbox.getMinimum()), eigenToQglviewer(bbox.getMaximum()));
}

void ViewerWidget::showDensePoints(const ViewpointPlanner::PointCloudType* dense_points) {
  dense_points_ = dense_points;
  if (!initialized_) {
      return;
  }

  std::cout << "Showing dense points" << std::endl;
  std::vector<rendering::OGLVertexDataRGBA> point_data;
  point_data.reserve(dense_points_->m_points.size());
  for (std::size_t i = 0; i < dense_points_->m_points.size(); ++i) {
    const auto& ml_vertex = dense_points_->m_points[i];
    const auto& ml_color = dense_points_->m_colors[i];
    rendering::OGLVertexDataRGBA point;
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
  using rendering::OGLLineData;
  using rendering::OGLVertexData;
  using rendering::OGLColorData;
  using rendering::OGLVertexDataRGBA;

  poisson_mesh_ = poisson_mesh;
  if (!initialized_) {
      return;
  }

  std::cout << "Showing poisson_mesh" << std::endl;
  std::vector<rendering::OGLTriangleData> triangle_data;
  triangle_data.reserve(poisson_mesh_->m_FaceIndicesVertices.size());
  const ml::BoundingBox3<FloatType> mesh_bbox = poisson_mesh_->computeBoundingBox();
  const bh::ColorMapJet<FloatType> cmap;
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const ViewpointPlanner::MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
    const ml::vec3f& v1 = poisson_mesh_->m_Vertices[face[0]];
    const ml::vec3f& v2 = poisson_mesh_->m_Vertices[face[1]];
    const ml::vec3f& v3 = poisson_mesh_->m_Vertices[face[2]];
    ml::vec4f c1;
    ml::vec4f c2;
    ml::vec4f c3;
    if (poisson_mesh->hasColors()) {
      c1 = poisson_mesh_->m_Colors[face[0]];
      c2 = poisson_mesh_->m_Colors[face[1]];
      c3 = poisson_mesh_->m_Colors[face[2]];
    }
    else {
      const FloatType coeff1 = (v1.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const FloatType coeff2 = (v2.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const FloatType coeff3 = (v3.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const bh::Color3<FloatType> color1 = cmap.map(coeff1);
      const bh::Color3<FloatType> color2 = cmap.map(coeff2);
      const bh::Color3<FloatType> color3 = cmap.map(coeff3);
      c1 = ml::vec4f(color1.r(), color1.g(), color1.b(), 1);
      c2 = ml::vec4f(color2.r(), color2.g(), color2.b(), 1);
      c3 = ml::vec4f(color3.r(), color3.g(), color3.b(), 1);
    }
    const FloatType a1 = 1;
    const FloatType a2 = 1;
    const FloatType a3 = 1;
    rendering::OGLTriangleData triangle;
    triangle.vertex1 = OGLVertexDataRGBA(v1.x, v1.y, v1.z, c1.r, c1.g, c1.b, a1);
    triangle.vertex2 = OGLVertexDataRGBA(v2.x, v2.y, v2.z, c2.r, c2.g, c2.b, a2);
    triangle.vertex3 = OGLVertexDataRGBA(v3.x, v3.y, v3.z, c3.r, c3.g, c3.b, a3);
    triangle_data.push_back(triangle);
  }
  std::cout << "Uploading " << triangle_data.size() << " triangles" << std::endl;
  poisson_mesh_drawer_.upload(triangle_data);

  std::vector<OGLLineData> line_data;
  const bh::Color4<FloatType> c(1, 0, 0, 1);
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const ViewpointPlanner::MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
    const ml::vec3f& ml_v1 = poisson_mesh_->m_Vertices[face[0]];
    const ml::vec3f& ml_v2 = poisson_mesh_->m_Vertices[face[1]];
    const ml::vec3f& ml_v3 = poisson_mesh_->m_Vertices[face[2]];
    const Vector3 v1(ml_v1.x, ml_v1.y, ml_v1.z);
    const Vector3 v2(ml_v2.x, ml_v2.y, ml_v2.z);
    const Vector3 v3(ml_v3.x, ml_v3.y, ml_v3.z);
    const Vector3 centroid = (v1 + v2 + v3) / 3;
    const Vector3 normal = FloatType(0.5) * (v1 - v2).cross(v2 - v3).normalized();
    OGLLineData line;
    line.vertex1 = OGLVertexDataRGBA(centroid.x(), centroid.y(), centroid.z(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(centroid.x() + normal.x(), centroid.y() + normal.y(), centroid.z() + normal.z(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
  }
  poisson_mesh_normal_drawer_.upload(line_data);
}

std::vector<rendering::OGLLineData> ViewerWidget::getRegionLineData(
        const ViewpointPlanner::RegionType& region, const bh::Color4<FloatType>& color) {
  using rendering::OGLLineData;
  using rendering::OGLVertexDataRGBA;
  const ViewpointPlanner::RegionType::Polygon2DType &polygon = region.getPolygon2D();
  std::vector<OGLLineData> line_data;
  line_data.reserve(polygon.getVertexCount() * 3);
  const bh::Color4<FloatType>& c = color;
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
    line.vertex1 = OGLVertexDataRGBA(v1(0), v1(1), region.getLowerPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(v2(0), v2(1), region.getLowerPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
    line.vertex1 = OGLVertexDataRGBA(v1(0), v1(1), region.getUpperPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(v2(0), v2(1), region.getUpperPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
    line.vertex1 = OGLVertexDataRGBA(v1(0), v1(1), region.getLowerPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line.vertex2 = OGLVertexDataRGBA(v1(0), v1(1), region.getUpperPlaneZ(), c.r(), c.g(), c.b(), c.a());
    line_data.push_back(line);
  }
  return line_data;
}

void ViewerWidget::showRegionOfInterest() {
  using rendering::OGLLineData;
  std::cout << "Showing region of interest" << std::endl;
  std::vector<OGLLineData> line_data;
  // Region of interest
  {
    const ViewpointPlanner::RegionType &roi = planner_->getRoi();
    const bh::Color4<FloatType> color(1, 1, 0, 1);
    const std::vector<OGLLineData> tmp_line_data = getRegionLineData(roi, color);
    line_data.insert(line_data.end(), tmp_line_data.begin(), tmp_line_data.end());
  }
  // No fly zones
  for (const ViewpointPlanner::RegionType& no_fly_zone : planner_->getNoFlyZones()) {
    const bh::Color4<FloatType> color(1, 0, 0, 1);
    const std::vector<OGLLineData> tmp_line_data = getRegionLineData(no_fly_zone, color);
    line_data.insert(line_data.end(), tmp_line_data.begin(), tmp_line_data.end());
  }
  std::cout << "Uploading " << line_data.size() << " lines" << std::endl;
  region_of_interest_drawer_.upload(line_data);
}

void ViewerWidget::showBvhBbox(const ViewpointPlanner::BoundingBoxType& bvh_bbox) {
  using rendering::OGLLineData;
  using rendering::OGLVertexDataRGBA;
  std::cout << "Showing BVH bounding box" << std::endl;
  std::vector<OGLLineData> line_data;
  line_data.reserve(4 * 3);
  const bh::Color4<FloatType> c(1, 0, 1, 1);
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
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif
  makeCurrent();

  // Fill viewpoint graph dropbox in planner panel
  std::vector<std::pair<std::string, size_t>> viewpoint_graph_gui_entries;
  for (size_t i = 0; i < planner_->getViewpointEntries().size(); ++i) {
    ViewpointPlanner::FloatType total_information = planner_->getViewpointEntries()[i].total_information;
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

  uploadViewpointGraphDrawerViewpointsWithoutLock(selected_index);

  planner_panel_->setViewpointGraphSize(planner_->getViewpointEntries().size());
  planner_panel_->setViewpointMotionsSize(planner_->getViewpointGraph().numEdges());

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  planner_lock.unlock();
  lock.unlock();

  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  const bool valid_selection = selected_index != (std::size_t)-1;
  if (inspect_graph_motions && valid_selection) {
    showViewpointGraphMotions(selected_index);
  }

  update();
}

void ViewerWidget::uploadViewpointGraphDrawerViewpointsWithoutLock(
        const size_t selected_index, const bool use_selection_mode) {
  std::vector<std::size_t> components;
  std::size_t num_components;
  std::tie(components, num_components) = planner_->getConnectedComponents();

  // Compute min-max of information value
  bh::MinMaxTracker<FloatType> min_max;
  for (const auto& entry : planner_->getViewpointEntries()) {
    FloatType total_information = entry.total_information;
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
  std::vector<Color4> colors;
  poses.reserve(planner_->getViewpointEntries().size());
  colors.reserve(planner_->getViewpointEntries().size());
  const bh::ColorMapHot<FloatType> cmap;
  const auto first_entry = planner_->getViewpointEntries().begin();
  const auto last_entry = planner_->getViewpointEntries().end();
  for (auto it = first_entry; it != last_entry; ++it) {
    const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = it - first_entry;
    const ViewpointPlanner::ViewpointEntry& entry = *it;
    if (viewpoint_selected_component_ >= 0
        && components[viewpoint_index] != (std::size_t)viewpoint_selected_component_) {
      continue;
    }
    FloatType total_information = entry.total_information;
    if (inspect_graph_motions && valid_selection) {
      if (viewpoint_index != selected_index
          && !planner_->getViewpointGraph().getEdgesByNode(selected_index).containsTargetNode(viewpoint_index)) {
        continue;
      }
    }
    const Pose& pose = entry.viewpoint.pose();
    Color4 color;
    if (use_selection_mode) {
      std::tie(std::ignore, color) = addSelectableObject(
              VIEWPOINT_GRAPH_ENTRY, boost::any((size_t)viewpoint_index));
    }
    else if (viewpoint_color_mode_ == Fixed
        && valid_selection && viewpoint_index == selected_index) {
      color = Color4(0.8f, 0.0f, 0.0f, 0.6f);
    }
    else if (viewpoint_color_mode_ == Fixed) {
      color = Color4(0.7f, 0.8f, 0.0f, 0.6f);
    }
    else if (viewpoint_color_mode_ == Component) {
      const std::size_t component = components[viewpoint_index];
      const FloatType value = bh::normalize<FloatType>(component, 0, num_components);
      color = cmap.map(value, 0.6f);
    }
    else if (viewpoint_color_mode_ == Indexed) {
      const FloatType value = bh::normalize<FloatType>(viewpoint_index, 0, planner_->getViewpointEntries().size() - 1);
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
  viewpoint_graph_drawer_.setCamera(planner_->getVirtualCamera());
  viewpoint_graph_drawer_.setViewpoints(poses, colors);
}

void ViewerWidget::showViewpointPath(const std::size_t selected_index /*= (std::size_t)-1*/) {
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif

  // Fill viewpoint path branch dropbox in planner panel
  std::vector<std::pair<std::string, size_t>> viewpoint_path_branch_gui_entries;
  for (size_t i = 0; i < planner_->getViewpointPaths().size(); ++i) {
    ViewpointPlanner::FloatType objective = planner_->getViewpointPaths()[i].acc_objective;
    std::string name = std::to_string(i) + " - " + std::to_string(objective);
    viewpoint_path_branch_gui_entries.push_back(std::make_pair(name, i));
  }
  planner_panel_->initializeViewpointPathBranch(viewpoint_path_branch_gui_entries);

  const size_t viewpoint_path_branch_index =
          (selected_viewpoint_path_branch_index_ != (size_t) -1) ? selected_viewpoint_path_branch_index_ : 0;
  const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index];
  // Fill viewpoint path dropbox in planner panel
  std::vector<std::pair<std::string, size_t>> viewpoint_path_gui_entries;
  for (size_t i = 0; i < viewpoint_path.entries.size(); ++i) {
    size_t path_entry_index = i;
    if (viewpoint_path.order.size() == viewpoint_path.entries.size()) {
      path_entry_index = viewpoint_path.order[i];
    }
    ViewpointPlanner::FloatType information = viewpoint_path.entries[path_entry_index].local_information;
    std::string name = std::to_string(path_entry_index) + " - " + std::to_string(information);
    viewpoint_path_gui_entries.push_back(std::make_pair(name, path_entry_index));
  }
  planner_panel_->initializeViewpointPath(viewpoint_path_gui_entries);

  planner_panel_->setViewpointPathSize(viewpoint_path.entries.size());

  uploadViewpointPathDrawerViewpointsWithoutLock(selected_index);

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  planner_lock.unlock();
  lock.unlock();

  showViewpointPathMotions(selected_index);

  update();
}

void ViewerWidget::uploadViewpointPathDrawerViewpointsWithoutLock(
        const size_t selected_index, const bool use_selection_mode) {
  const size_t viewpoint_path_branch_index =
          (selected_viewpoint_path_branch_index_ != (size_t) -1) ? selected_viewpoint_path_branch_index_ : 0;
  const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index];
  // Compute min-max of information value
  bh::MinMaxTracker<FloatType> min_max;
  for (const auto& entry : viewpoint_path.entries) {
    const FloatType total_information = entry.local_information;
    min_max.update(total_information);
  }

  // Get viewpoint path and selection/motion inspection flags
  const bool valid_selection = selected_index != (std::size_t)-1;
  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();

  // Retrieve connected components data
  std::vector<std::size_t> component;
  std::size_t num_components;
  std::tie(component, num_components) = planner_->getConnectedComponents();

  // Generate OGL data for viewpoint drawer
  std::vector<Pose> poses;
  std::vector<Color4> colors;
  const bh::ColorMapHot<FloatType> cmap;
  ViewpointPlanner::ViewpointEntryIndex selected_viewpoint_index = (std::size_t)-1;
  if (valid_selection) {
    selected_viewpoint_index = viewpoint_path.entries[selected_index].viewpoint_index;
  }
  for (auto it = viewpoint_path.entries.begin(); it != viewpoint_path.entries.end(); ++it) {
    const ViewpointPlanner::ViewpointPathEntry& path_entry = *it;
    const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[path_entry.viewpoint_index];
    const FloatType total_information = viewpoint_entry.total_information;
    if (inspect_graph_motions && valid_selection) {
      // If motions are inspected, only draw selected viewpoint or neighbouring viewpoints
      if (selected_viewpoint_index != path_entry.viewpoint_index) {
        const auto order_it = std::find(viewpoint_path.order.begin(), viewpoint_path.order.end(), selected_index);
        if (order_it == viewpoint_path.order.end()) {
          continue;
        }
        auto next_order_it = order_it + 1;
        if (next_order_it == viewpoint_path.order.end()) {
          next_order_it = viewpoint_path.order.begin();
        }
        if (path_entry.viewpoint_index != viewpoint_path.entries[*next_order_it].viewpoint_index) {
          continue;
        }
      }
    }

    const Pose& pose = path_entry.viewpoint.pose();
    const bool mvs_viewpoint = path_entry.mvs_viewpoint;
    Color4 color;
    if (use_selection_mode) {
      std::tie(std::ignore, color) = addSelectableObject(
              VIEWPOINT_PATH_ENTRY, boost::any(size_t(it - viewpoint_path.entries.begin())));
    }
    else if (viewpoint_color_mode_ == Fixed) {
      if (valid_selection && path_entry.viewpoint_index == selected_viewpoint_index) {
        color = Color4(0.8f, 0.0f, 0.0f, 0.6f);
      }
      else if (mvs_viewpoint) {
        color = Color4(0.0f, 0.1f, 0.8f, 0.6f);
      }
      else {
        color = Color4(0.0f, 0.8f, 0.8f, 0.6f);
      }
    }
    else if (viewpoint_color_mode_ == Component) {
      std::size_t comp = component[path_entry.viewpoint_index];
      const FloatType value = bh::normalize<FloatType>(comp, 0, num_components);
      color = cmap.map(value, 0.6f);
    }
    else if (viewpoint_color_mode_ == Indexed) {
      const std::size_t entry_index = it - viewpoint_path.entries.begin();
      const FloatType value = bh::normalize<FloatType>(entry_index, 0, viewpoint_path.entries.size() - 1);
      color = cmap.map(value, 0.6f);
    }
    else {
      const FloatType value = min_max.normalize(total_information);
      color = cmap.map(value, 0.6f);
    }
    poses.push_back(pose);
    colors.push_back(color);
  }
  if (viewpoint_path.order.size() > 1) {
    for (auto order_it = viewpoint_path.order.begin(); order_it != viewpoint_path.order.end(); ++order_it) {
      if (inspect_graph_motions && valid_selection && *order_it != selected_index) {
        continue;
      }
      // Show sparse matching viewpoints
      auto next_order_it = order_it + 1;
      if (next_order_it == viewpoint_path.order.end()) {
        next_order_it = viewpoint_path.order.begin();
      }
      const size_t path_entry_index = *order_it;
      const size_t next_path_entry_index = *next_order_it;
      const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[path_entry_index].viewpoint_index;
      const ViewpointPlanner::ViewpointEntryIndex next_viewpoint_index = viewpoint_path.entries[next_path_entry_index].viewpoint_index;
      if (planner_->hasViewpointMotion(viewpoint_index, next_viewpoint_index)) {
        const ViewpointPlanner::ViewpointMotion &motion = planner_->getViewpointMotion(viewpoint_index,
                                                                                       next_viewpoint_index);

        for (auto it = motion.viewpointIndices().begin() + 1; it != motion.viewpointIndices().end() - 1; ++it) {
          Color4 color(0.0f, 0.8f, 0.1f, 0.6f);
          const Pose &pose = planner_->getViewpointEntries()[*it].viewpoint.pose();
          poses.push_back(pose);
          colors.push_back(color);
        }
        if (inspect_graph_motions && valid_selection) {
          // Print matchability matrix along motion
          std::cout << "Matchability matrix:" << std::endl;
          for (auto it = motion.viewpointIndices().begin(); it != motion.viewpointIndices().end(); ++it) {
            for (auto it2 = it + 1; it2 != motion.viewpointIndices().end(); ++it2) {
              //            const bool matchable = planner_->isSparseMatchable(*it, *it2);
              //            const bool matchable = planner_->isSparseMatchable2(*it, *it2);
              const bool matchable = false;
              const size_t motion_index1 = it - motion.viewpointIndices().begin();
              const size_t motion_index2 = it2 - motion.viewpointIndices().begin();
              std::cout << "  " << motion_index1 << "[" << *it << "] <-> " << motion_index2 << " [" << *it2
                        << "] matchable: " << matchable << std::endl;
            }
          }
        }
      }
    }
  }

  // Draw viewpoint camera frustums
  viewpoint_path_drawer_.setCamera(planner_->getVirtualCamera());
  viewpoint_path_drawer_.setViewpoints(poses, colors);
}

ViewerWidget::Color4 ViewerWidget::readPixelColor(const size_t x, const size_t y) const {
  const size_t x_scaled = static_cast<size_t>(devicePixelRatio() * x);
  const size_t y_scaled = static_cast<size_t>(devicePixelRatio() * (height() - y - 1));
  Color4 color;
  glReadPixels(x_scaled, y_scaled, 1, 1, GL_RGBA, GL_FLOAT, color.data());
//  BH_PRINT_VALUE(x_scaled);
//  BH_PRINT_VALUE(y_scaled);
//  BH_PRINT_VALUE(color);
  return color;
}

size_t ViewerWidget::colorToSelectionIndex(const Color4& color) const {
//  BH_PRINT_VALUE(static_cast<size_t>(color.r() * FloatType(255)) << 0);
//  BH_PRINT_VALUE(static_cast<size_t>(color.g() * FloatType(255)) << 8);
//  BH_PRINT_VALUE(static_cast<size_t>(color.b() * FloatType(255)) << 16);
  return (static_cast<size_t>(color.r() * FloatType(255)) << 0) +
          (static_cast<size_t>(color.g() * FloatType(255)) << 8) +
          (static_cast<size_t>(color.b() * FloatType(255)) << 16);
}

ViewerWidget::Color4 ViewerWidget::selectionIndexToColor(const size_t index) const {
  const FloatType r = ((index & 0x000000FF) >> 0) / FloatType(255);
  const FloatType g = ((index & 0x0000FF00) >> 8) / FloatType(255);
  const FloatType b = ((index & 0x00FF0000) >> 16) / FloatType(255);
  const FloatType a = 1;
  return Color4(r, g, b, a);
}

void ViewerWidget::showViewpointGraphMotions(const std::size_t selected_index) {
  using rendering::OGLLineData;
  using rendering::OGLColorData;
  using rendering::OGLVertexData;
  using rendering::OGLVertexDataRGBA;
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  // Draw motion path
  std::vector<OGLLineData> lines;
//  const OGLColorData line_color(0.7f, 0.1f, 0.0f, 1.0f);
  const bh::ColorMapHot<FloatType> cmap;
  ViewpointPlanner::ViewpointGraph& viewpoint_graph = planner_->getViewpointGraph();

  std::cout << "Motions:" << std::endl;
  auto edges = viewpoint_graph.getEdgesByNode(selected_index);
  std::size_t edge_index = 0;
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    const ViewpointPlanner::ViewpointEntryIndex from_index = selected_index;
    const ViewpointPlanner::ViewpointEntryIndex to_index = it.targetNode();
    const ViewpointPlanner::ViewpointMotion& motion = planner_->getViewpointMotion(from_index, to_index);
//    const bool matchable = planner_->isSparseMatchable(from_index, to_index);
//    const bool matchable = planner_->isSparseMatchable2(from_index, to_index);
    const bool matchable = true;
    std::cout << "  -> " << to_index << ", distance=" << motion.distance()
        << ", matchable=" << matchable << std::endl;
    // Careful, assuming random access iterators
    FloatType total_distance = motion.distance();
    FloatType acc_dist = 0;
    for (auto se3_motion_it = motion.se3Motions().begin(); se3_motion_it != motion.se3Motions().end(); ++se3_motion_it) {
      BH_ASSERT(!se3_motion_it->poses().empty());
      for (auto pose_it = se3_motion_it->poses().begin() + 1; pose_it != se3_motion_it->poses().end(); ++pose_it) {
        const Vector3 from_pos = (pose_it - 1)->getWorldPosition();
        const Vector3 to_pos = pose_it->getWorldPosition();
        const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
        const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
        const FloatType local_dist = (from_pos - to_pos).norm();
        bh::Color3<FloatType> color1;
        bh::Color3<FloatType> color2;
        if (viewpoint_color_mode_ == Indexed) {
          const FloatType value = bh::normalize<FloatType>(edge_index, 0, edges.size() - 1);
          color1 = cmap.map(value);
          color2 = cmap.map(value);
        }
        else {
          const FloatType value1 = bh::normalize<FloatType>(acc_dist, 0, total_distance);
          const FloatType value2 = bh::normalize<FloatType>(acc_dist + local_dist, 0, total_distance);
          color1 = cmap.map(value1);
          color2 = cmap.map(value2);
        }
        const OGLColorData line_color1(color1.r(), color1.g(), color1.b(), 1.0f);
        const OGLColorData line_color2(color2.r(), color2.g(), color2.b(), 1.0f);
        lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color1), OGLVertexDataRGBA(vertex2, line_color2));
        acc_dist += local_dist;
      }
    }
    ++edge_index;
  }
  std::cout << "Selected viewpoint " << selected_index << " has " << edges.size() << " edges" << std::endl;

#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif
  makeCurrent();
  viewpoint_motion_line_drawer_.upload(lines);

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  lock.unlock();

  update();
}

void ViewerWidget::showViewpointPathMotions(const std::size_t selected_index /*= (std::size_t)-1*/) {
  using rendering::OGLLineData;
  using rendering::OGLColorData;
  using rendering::OGLVertexData;
  using rendering::OGLVertexDataRGBA;

  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif

  makeCurrent();

  const ViewpointPlanner::ViewpointPath viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index_];

  const bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  const bool valid_selection = selected_index != (std::size_t)-1;
  const bh::ColorMapHot<FloatType> cmap;
  // Draw motion path
  std::vector<OGLLineData> lines;
  if (inspect_graph_motions && valid_selection) {
    if (viewpoint_path.order.empty()) {
      std::cout << "Warning: Viewpoint path has no associated tour" << std::endl;
    }
    else {
      const OGLColorData line_color(0.7f, 0.1f, 0.0f, 1.0f);
      const auto order_it = std::find(viewpoint_path.order.begin(), viewpoint_path.order.end(), selected_index);
      auto next_order_it = order_it + 1;
      if (next_order_it == viewpoint_path.order.end()) {
        next_order_it = viewpoint_path.order.begin();
      }
      const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[selected_index].viewpoint_index;
      const ViewpointPlanner::ViewpointEntryIndex next_viewpoint_index = viewpoint_path.entries[*next_order_it].viewpoint_index;
      const ViewpointPlanner::ViewpointMotion& motion = planner_->getViewpointMotion(viewpoint_index, next_viewpoint_index);
      // Careful, assuming random access iterators
      FloatType total_distance = motion.distance();
      FloatType acc_dist = 0;
      for (auto se3_motion_it = motion.se3Motions().begin(); se3_motion_it != motion.se3Motions().end(); ++se3_motion_it) {
        BH_ASSERT(!se3_motion_it->poses().empty());
        for (auto pose_it = se3_motion_it->poses().begin() + 1; pose_it != se3_motion_it->poses().end(); ++pose_it) {
          const Vector3 from_pos = (pose_it - 1)->getWorldPosition();
          const Vector3 to_pos = pose_it->getWorldPosition();
          const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
          const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
          const FloatType local_dist = (from_pos - to_pos).norm();
          const FloatType value1 = bh::normalize<FloatType>(acc_dist, 0, total_distance);
          const FloatType value2 = bh::normalize<FloatType>(acc_dist + local_dist, 0, total_distance);
          acc_dist += local_dist;
          bh::Color3<FloatType> color1 = cmap.map(value1);
          const OGLColorData line_color1(color1.r(), color1.g(), color1.b(), 1.0f);
          bh::Color3<FloatType> color2 = cmap.map(value2);
          const OGLColorData line_color2(color2.r(), color2.g(), color2.b(), 1.0f);
          lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color1), OGLVertexDataRGBA(vertex2, line_color2));
        }
      }
    }
  }
  else if (viewpoint_path.entries.size() > 1) {
    // Compute total distance of path
    FloatType total_distance = 0;
    for (auto it = viewpoint_path.order.begin(); it != viewpoint_path.order.end(); ++it) {
      const ViewpointPlanner::ViewpointEntryIndex from_index = viewpoint_path.entries[*it].viewpoint_index;
      auto next_it = it + 1;
      if (next_it == viewpoint_path.order.end()) {
        next_it = viewpoint_path.order.begin();
      }
      const ViewpointPlanner::ViewpointEntryIndex to_index = viewpoint_path.entries[*next_it].viewpoint_index;
      if (planner_->hasViewpointMotion(from_index, to_index)) {
        const ViewpointPlanner::ViewpointMotion& motion = planner_->getViewpointMotion(from_index, to_index);
        total_distance += motion.distance();
      }
    }

    FloatType acc_dist = 0;
    for (auto it = viewpoint_path.order.begin(); it != viewpoint_path.order.end(); ++it) {
      const ViewpointPlanner::ViewpointEntryIndex from_index = viewpoint_path.entries[*it].viewpoint_index;
      auto next_it = it + 1;
      if (next_it == viewpoint_path.order.end()) {
        next_it = viewpoint_path.order.begin();
      }
      const ViewpointPlanner::ViewpointEntryIndex to_index = viewpoint_path.entries[*next_it].viewpoint_index;
      if (!planner_->hasViewpointMotion(from_index, to_index)) {
        continue;
      }
      const ViewpointPlanner::ViewpointMotion& motion = planner_->getViewpointMotion(from_index, to_index);
      for (auto se3_motion_it = motion.se3Motions().begin(); se3_motion_it != motion.se3Motions().end(); ++se3_motion_it) {
        BH_ASSERT(!se3_motion_it->poses().empty());
        for (auto pose_it = se3_motion_it->poses().begin() + 1; pose_it != se3_motion_it->poses().end(); ++pose_it) {
          const Vector3 from_pos = (pose_it - 1)->getWorldPosition();
          const Vector3 to_pos = pose_it->getWorldPosition();
          const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
          const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
          const FloatType local_dist = (from_pos - to_pos).norm();
          acc_dist += local_dist;
          bh::Color3<FloatType> color1;
          bh::Color3<FloatType> color2;
          if (viewpoint_color_mode_ == Indexed) {
            const std::size_t entry_index = it - viewpoint_path.order.begin();
            const FloatType value = bh::normalize<FloatType>(entry_index, 0, viewpoint_path.order.size() - 1);
            color1 = cmap.map(value);
            color2 = cmap.map(value);
          }
          else {
            const FloatType value1 = bh::normalize<FloatType>(acc_dist, 0, total_distance);
            const FloatType value2 = bh::normalize<FloatType>(acc_dist + local_dist, 0, total_distance);
            color1 = cmap.map(value1);
            color2 = cmap.map(value2);
          }
          const OGLColorData line_color1(color1.r(), color1.g(), color1.b(), 1.0f);
          const OGLColorData line_color2(color2.r(), color2.g(), color2.b(), 1.0f);
          lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color1), OGLVertexDataRGBA(vertex2, line_color2));
        }
      }
    }
  }
  viewpoint_motion_line_drawer_.upload(lines);

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  planner_lock.unlock();
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

    sparse_recon_drawer_.overwriteCameraWidth(planner_->getVirtualCamera().width());
    sparse_recon_drawer_.setSparseReconstruction(sparse_recon_);

    update();
}

void ViewerWidget::refreshTree()
{
    if (octree_ != nullptr) {
        octree_drawer_.setOctree(
                octree_,
                std::max(planner_->getBvhBbox().getMinimum(2), options_.ground_height),
                std::min(planner_->getBvhBbox().getMaximum(2), planner_->getPlannerData().getOptions().obstacle_free_height));
    }
    update();
}

void ViewerWidget::setDrawRaycast(bool draw_raycast) {
  octree_drawer_.setDrawRaycast(draw_raycast);
  update();
}

void ViewerWidget::captureRaycast() {
  const Pose camera_pose = getCameraPose();
  const Viewpoint viewpoint = planner_->getVirtualViewpoint(camera_pose);
  raycast_mode_ = RaycastMode::DEFAULT;
//  std::cout << "raycast pose: " << camera_pose << std::endl;
//  std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, FloatType>> raycast_results = planner_->getRaycastHitVoxels(camera_pose);
  std::cout << "BVH bounding box: " << planner_->getBvhTree().getRoot()->getBoundingBox() << std::endl;
  planner_thread_.requestRaycast(viewpoint, 0, viewpoint.camera().width(), 0, viewpoint.camera().height());
}

void ViewerWidget::captureRaycastWithCurrentInformation() {
  const Pose camera_pose = getCameraPose();
  const Viewpoint viewpoint = planner_->getVirtualViewpoint(camera_pose);
  raycast_mode_ = RaycastMode::WITH_CURRENT_INFORMATION;
  planner_thread_.requestRaycast(viewpoint, 0, viewpoint.camera().width(), 0, viewpoint.camera().height());
}

void ViewerWidget::captureInformationVoxelCenter() {
  const Pose camera_pose = getCameraPose();
  const Viewpoint viewpoint = planner_->getVirtualViewpoint(camera_pose);
  raycast_mode_ = RaycastMode::INFORMATION_VOXEL_CENTER;
  planner_thread_.requestRaycast(viewpoint, 0, viewpoint.camera().width(), 0, viewpoint.camera().height());
}

void ViewerWidget::captureRaycast(
    const std::size_t x_start, const std::size_t x_end, const std::size_t y_start, const std::size_t y_end) {
  const Pose camera_pose = getCameraPose();
  const Viewpoint viewpoint = planner_->getVirtualViewpoint(camera_pose);
  raycast_mode_ = RaycastMode::DEFAULT;
  planner_thread_.requestRaycast(viewpoint, x_start, x_end, y_start, y_end);
}

void ViewerWidget::captureRaycastWindow(const std::size_t width, const std::size_t height) {
  const Pose camera_pose = getCameraPose();
  const Viewpoint viewpoint = planner_->getVirtualViewpoint(camera_pose);
  raycast_mode_ = RaycastMode::DEFAULT;
  const PinholeCamera& camera = viewpoint.camera();
//  std::cout << "raycast pose: " << camera_pose << std::endl;
//  std::pair<ViewpointPlanner::VoxelWithInformationSet, FloatType> raycast_results =
//      planner_->getRaycastHitVoxelsWithInformationScore(viewpoint, width, height);
  std::size_t y_start = camera.height() / 2 - height / 2;
  std::size_t y_end = camera.height() / 2 + height / 2 + 1;
  std::size_t x_start = camera.width() / 2 - width / 2;
  std::size_t x_end = camera.width() / 2 + width / 2 + 1;
  y_start = bh::clamp<std::size_t>(y_start, 0, camera.height());
  y_end = bh::clamp<std::size_t>(y_end, 0, camera.height());
  x_start = bh::clamp<std::size_t>(x_start, 0, camera.width());
  x_end = bh::clamp<std::size_t>(x_end, 0, camera.width());
  planner_thread_.requestRaycast(viewpoint, x_start, x_end, y_start, y_end);
}

void ViewerWidget::runInPlannerThread(const std::function<void()>& function) {
  custom_request_finished_handler_ = std::function<void()>();
  planner_thread_.customRequest(function);
}

void ViewerWidget::runInPlannerThread(const std::function<void()>& function,
                                      const std::function<void()>& finished_handler) {
  std::cout << "Running operation in planner thread" << std::endl;
  custom_request_finished_handler_ = finished_handler;
  planner_thread_.customRequest(function);
}

void ViewerWidget::runInPlannerThreadAndWait(const std::function<void()>& function) {
  custom_request_finished_handler_ = std::function<void()>();
  planner_thread_.customRequest(function);
  planner_thread_.waitForCustomRequest();
}

void ViewerWidget::runInPlannerThreadAndWait(const std::function<void()>& function,
                                      const std::function<void()>& finished_handler) {
  std::cout << "Running operation in planner thread" << std::endl;
  custom_request_finished_handler_ = std::function<void()>();
  planner_thread_.customRequest(function);
  planner_thread_.waitForCustomRequest();
  finished_handler();
}

void ViewerWidget::onRaycastFinished() {
  const Viewpoint& viewpoint = planner_thread_.getRaycastViewpoint();
  const std::size_t width = planner_thread_.getRaycastXEnd() - planner_thread_.getRaycastXStart();
  const std::size_t height = planner_thread_.getRaycastYEnd() - planner_thread_.getRaycastYStart();
  const std::pair<ViewpointPlanner::VoxelWithInformationSet, FloatType>& raycast_results = planner_thread_.getRaycastResults();
  const std::unordered_map<ViewpointPlanner::VoxelWrapper, Vector3, ViewpointPlanner::VoxelWrapper::Hash>&
    poisson_mesh_normals = planner_thread_.getRaycastPoissonMeshNormals();
  const std::unordered_map<ViewpointPlanner::VoxelWrapper, FloatType, ViewpointPlanner::VoxelWrapper::Hash>&
    poisson_mesh_depth = planner_thread_.getRaycastPoissonMeshDepth();
  const std::unordered_map<ViewpointPlanner::VoxelWrapper, Vector2, ViewpointPlanner::VoxelWrapper::Hash>&
    screen_coordinates = planner_thread_.getRaycastScreenCoordinates();
  const ViewpointPlanner::VoxelWithInformationSet& raycast_voxels = raycast_results.first;
//#if WITH_CUDA
////  // Test code for Cuda raycast
////  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results_non_cuda
////    = planner_->getRaycastHitVoxels(camera_pose, width, height);
////  std::cout << "Non-cuda raycast hit " << raycast_results_non_cuda.size() << " voxels" << std::endl;
////  // End of test code
//  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results
//    = planner_->getRaycastHitVoxelsCuda(viewpoint, width, height);
//#else
//  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results
//    = planner_->getRaycastHitVoxels(camera_pose, width, height);
//#endif
  if (width < viewpoint.camera().width() && raycast_mode_ == RaycastMode::DEFAULT) {
    std::cout << "Raycast hit " << raycast_voxels.size() << " voxels" << std::endl;
    std::cout << "Nodes:" << std::endl;
    for (auto it = raycast_voxels.begin(); it != raycast_voxels.end(); ++it) {
      const Vector3 position = it->voxel->getBoundingBox().getCenter();
      std::cout << " node: " << it->voxel << std::endl;
      std::cout << "  position=" << position.transpose() << std::endl;
      std::cout << "  distance=" << (position - viewpoint.pose().getWorldPosition()).norm() << std::endl;
      if (planner_->hasGpsTransformation()) {
        std::cout << "  GPS = " << getGpsFromPosition(position) << std::endl;
      }
      std::cout << "  screen coordinate=" << screen_coordinates.at(it->voxel).transpose() << std::endl;
      std::cout << "  normal field=" << it->voxel->getObject()->normal.transpose() << std::endl;
      std::cout << "  normal=" << poisson_mesh_normals.at(it->voxel).transpose() << std::endl;
      std::cout << "  depth=" << poisson_mesh_depth.at(it->voxel) << std::endl;
      std::cout << "  size=" << it->voxel->getBoundingBox().getMaxExtent() << std::endl;
      std::cout << "  occupancy=" << it->voxel->getObject()->occupancy << std::endl;
      std::cout << "  observation count=" << it->voxel->getObject()->observation_count << std::endl;
      std::cout << "  weight=" << it->voxel->getObject()->weight << std::endl;
      std::cout << "  information=" << it->information << std::endl;
      std::cout << "  ROI inside=" << planner_->getRoiBbox().isInside(position) << std::endl;
      std::cout << "  ROI distance=" << planner_->getRoiBbox().distanceTo(position) << std::endl;
      if (planner_->getPlannerData().getOptions().use_distance_field
        && planner_->getPlannerData().isInsideGrid(position)) {
        const Eigen::Vector3i grid_indices = planner_->getPlannerData().getGridIndices(position);
        std::cout << "  Distance field="
                  << planner_->getPlannerData().getDistanceField()(grid_indices(0), grid_indices(1), grid_indices(2))
                  << std::endl;
      }
      else {
        std::cout << "  Outside distance field" << std::endl;
      }
    }
//  std::cout << "Node with weight > 0.5" << std::endl;
//  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
//    if (it->node->getObject()->weight > 0.5) {
//      std::cout << "  &node_idx=" << planner_->getBvhTree().getVoxelIndexMap().at(it->node)
//          << ", weight=" << it->node->getObject()->weight
//          << ", obs_count=" << it->node->getObject()->observation_count << std::endl;
//    }
//  }
  }

  std::cout << "Total number of voxels: " << raycast_voxels.size() << std::endl;
  const FloatType total_information = std::accumulate(raycast_voxels.begin(), raycast_voxels.end(),
  FloatType { 0 }, [&](const FloatType& value, const ViewpointPlanner::VoxelWithInformation& vi) {
    const FloatType information = vi.information;
    return value + information;
  });
  std::cout << "Total information: " << total_information << std::endl;
  const ViewpointPlanner::ViewpointPath* viewpoint_path;
  if (selected_viewpoint_path_branch_index_ == (std::size_t)-1) {
    viewpoint_path = &planner_->getBestViewpointPath();
  }
  else {
    viewpoint_path = &planner_->getViewpointPaths()[selected_viewpoint_path_branch_index_];
  }
  if (raycast_mode_ == RaycastMode::INFORMATION_VOXEL_CENTER) {
    Vector3 voxel_center = Vector3::Zero();
    FloatType total_weight = 0;
    for (const ViewpointPlanner::VoxelWithInformation& vi : raycast_voxels) {
      const FloatType information = vi.information;
      voxel_center += information * vi.voxel->getBoundingBox().getCenter();
      total_weight += information;
    }
    voxel_center /= total_weight;
    std::cout << "Information voxel center: " << voxel_center.transpose() << std::endl;
    std::vector<std::pair<const ViewpointPlanner::VoxelType*, FloatType>> tmp;
    for (auto it = raycast_voxels.begin(); it != raycast_voxels.end(); ++it) {
  //    const FloatType information = planner_->computeViewpointObservationScore(viewpoint, it->voxel);
      FloatType information = it->information;
      if (raycast_mode_ == RaycastMode::WITH_CURRENT_INFORMATION) {
        const auto observed_it = viewpoint_path->observed_voxel_map.find(it->voxel);
        if (observed_it != viewpoint_path->observed_voxel_map.end()) {
          const FloatType voxel_weight = observed_it->first.voxel->getObject()->weight;
          information = std::min(information, voxel_weight - observed_it->second);
        }
      }
      tmp.push_back(std::make_pair(it->voxel, information));
    }
    using rendering::OGLVertexData;
    using rendering::OGLColorData;
    using rendering::OGLVoxelData;
    using rendering::OGLVoxelInfoData;
    OGLVoxelData voxel(OGLVertexData(voxel_center(0), voxel_center(1), voxel_center(2)), 0.1f);
    OGLColorData color(1, 0, 0, 1);
    OGLVoxelInfoData info(1.0f, 100.0f, 1.0f, 1.0f);
    octree_drawer_.updateSingleRaycastVoxel(voxel, color, info);
  }
  else {
    std::size_t new_voxels = 0;
    const FloatType new_information = std::accumulate(raycast_voxels.begin(), raycast_voxels.end(),
    FloatType { 0 }, [&](const FloatType& value, const ViewpointPlanner::VoxelWithInformation& vi) {
      FloatType information = vi.information;
      auto it = viewpoint_path->observed_voxel_map.find(vi.voxel);
      if (it != viewpoint_path->observed_voxel_map.end()) {
        information -= it->second;
        if (information < 0) {
          information = 0;
        }
      }
      else {
        ++new_voxels;
      }
      return value + information;
    });
    std::cout << "New voxels: " << new_voxels << std::endl;
    std::cout << "New information: " << new_information << std::endl;

    std::vector<std::pair<const ViewpointPlanner::VoxelType*, FloatType>> tmp;
    tmp.reserve(raycast_voxels.size());
    for (auto it = raycast_voxels.begin(); it != raycast_voxels.end(); ++it) {
  //    const FloatType information = planner_->computeViewpointObservationScore(viewpoint, it->voxel);
      FloatType information = it->information;
      if (raycast_mode_ == RaycastMode::WITH_CURRENT_INFORMATION) {
        const auto observed_it = viewpoint_path->observed_voxel_map.find(it->voxel);
        if (observed_it != viewpoint_path->observed_voxel_map.end()) {
          const FloatType voxel_weight = observed_it->first.voxel->getObject()->weight;
          information = std::min(information, voxel_weight - observed_it->second);
        }
      }
      tmp.push_back(std::make_pair(it->voxel, information));
    }
    octree_drawer_.updateRaycastVoxels(tmp);
    octree_drawer_.setInformationRange(0, 1);
    octree_drawer_.setWeightRange(0, 1);
    if (width == 0 && height == 0) {
      if (raycast_voxels.empty()) {
        sendClearSelectedPositionToWebSocketClients();
      }
      else {
        sendSelectedPositionToWebSocketClients(raycast_voxels.begin()->voxel->getBoundingBox().getCenter());
      }
    }
  }
  update();
}

void ViewerWidget::onMakeViewpointMotionsSparseMatchableFinished() {
  showViewpointGraph();
  showViewpointPath();
  update();
}

void ViewerWidget::onCustomRequestFinished() {
  if (custom_request_finished_handler_) {
    custom_request_finished_handler_();
  }
}

void ViewerWidget::signalRaycastFinished() {
  emit raycastFinished();
}

void ViewerWidget::signalMakeViewpointMotionsSparseMatchableFinished() {
  emit makeViewpointMotionsSparseMatchableFinished();
}

void ViewerWidget::signalMatchCameraPosesFinished() {
  emit matchCameraPosesFinished();
}

void ViewerWidget::signalCustomRequestFinished() {
  emit customRequestFinished();
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

void ViewerWidget::setOccupancyBinThreshold(double occupancy_threshold) {
//    std::cout << "Setting occupancy threshold to " << occupancy_threshold << std::endl;
    octree_drawer_.setOccupancyBinThreshold(occupancy_threshold);
    update();
}

void ViewerWidget::setColorFlags(uint32_t color_flags) {
//    std::cout << "Setting color flags to " << color_flags << std::endl;
    octree_drawer_.setColorFlags(color_flags);
    update();
}

void ViewerWidget::setDrawFreeVoxels(bool draw_free_voxels) {
    octree_drawer_.setDrawFreeVoxels(draw_free_voxels);
    update();
}

void ViewerWidget::setDrawAxes(bool draw_axes) {
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
  sparse_recon_drawer_.setDrawCameras(draw_cameras);
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
  sparse_recon_drawer_.setDrawSparsePoints(draw_sparse_points);
  update();
}

void ViewerWidget::setDrawDensePoints(bool draw_dense_points) {
  dense_points_drawer_.setDrawPoints(draw_dense_points);
  update();
}

void ViewerWidget::setDrawPoissonMesh(bool draw_poisson_mesh) {
  poisson_mesh_drawer_.setDrawTriangles(draw_poisson_mesh);
  if (options_.show_poisson_mesh_normals) {
    poisson_mesh_normal_drawer_.setDrawLines(draw_poisson_mesh);
  }
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
//    const reconstruction::PinholeCameraColmap& virtual_camera = sparse_recon_->getCameras().cbegin()->second;
    const reconstruction::PinholeCamera& virtual_camera = planner_->getVirtualCamera();
    double fy = virtual_camera.getFocalLengthY();
    double v_fov = 2 * std::atan(virtual_camera.height() / (2 * fy));
    camera()->setFieldOfView(v_fov);
    aspect_ratio_ = virtual_camera.width() / static_cast<double>(virtual_camera.height());
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
  std::cout << "  Position=(" << image.pose().getWorldPosition().transpose() << ")" << std::endl;
  std::cout << "  Orientation=" << image.pose().quaternion() << std::endl;
  std::cout << "  GPS=" << getGpsFromPose(image.pose()) << std::endl;
  update();
}

void ViewerWidget::setViewpointGraphSelectionIndex(const std::size_t index) {
  std::unique_lock<std::mutex> lock(mutex_);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif

  makeCurrent();

  selected_viewpoint_graph_entry_index_ = index;

  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = index;
  std::cout << "Selected viewpoint " << viewpoint_index << std::endl;
  const Pose& pose = planner_->getViewpointEntries()[viewpoint_index].viewpoint.pose();
  std::cout << "  Position=" << pose.getWorldPosition().transpose() << std::endl;
  std::cout << "  Orientation=" << pose.quaternion() << std::endl;
  if (planner_->hasGpsTransformation()) {
    std::cout << "  GPS=" << getGpsFromPose(pose) << std::endl;
  }
  const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
  const FloatType total_information = viewpoint_entry.total_information;
  std::cout << "  Total voxels=" << viewpoint_entry.voxel_set.size() << std::endl;
  std::cout << "  Total information=" << total_information << std::endl;

  if (selected_viewpoint_path_branch_index_ != (size_t)-1) {
    const ViewpointPlanner::ViewpointPath &viewpoint_path = planner_->getViewpointPaths()[selected_viewpoint_path_branch_index_];
    // Compute new information of selected viewpoint
    std::size_t new_voxels = 0;
    FloatType new_information = std::accumulate(viewpoint_entry.voxel_set.cbegin(), viewpoint_entry.voxel_set.cend(),
                                                FloatType {0}, [&](const FloatType &value,
                                                                   const ViewpointPlanner::VoxelWithInformation &voxel_with_information) {
              FloatType information = voxel_with_information.information;
              auto it = viewpoint_path.observed_voxel_map.find(voxel_with_information.voxel);
              if (it != viewpoint_path.observed_voxel_map.end()) {
                information -= it->second;
                if (information < 0) {
                  information = 0;
                }
              }
              else {
                ++new_voxels;
              }
              return value + information;
            });
    std::cout << "  New voxels=" << new_voxels << std::endl;
    std::cout << "  New information=" << new_information << std::endl;
  }
  if (planner_panel_->isUpdateCameraOnSelectionChecked()) {
    setCameraPose(viewpoint_entry.viewpoint.pose());
  }
  octree_drawer_.updateRaycastVoxels(viewpoint_entry.voxel_set);
  octree_drawer_.setInformationRange(0, 1);
  octree_drawer_.setWeightRange(0, 1);

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  planner_lock.unlock();
  lock.unlock();

  showViewpointGraph(index);
  planner_panel_->setViewpointGraphSelection(index);
}

void ViewerWidget::setViewpointPathBranchSelectionIndex(std::size_t index) {
  std::unique_lock<std::mutex> lock(mutex_);

  selected_viewpoint_path_branch_index_ = index;

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
  std::cout << "Order:" << std::endl;
  for (std::size_t i = 0; i < viewpoint_path.order.size(); ++i) {
    std::cout << "  " << i << ": " << viewpoint_path.order[i]
              << ", entry=" << viewpoint_path.entries[viewpoint_path.order[i]].viewpoint_index << std::endl;
  }
  lock.unlock();
  // Remember selected entry before updating combo box
  const std::size_t path_selection_index = planner_panel_->getViewpointPathSelection();
  showViewpointPath();
  // Set previous selection for combo box
  planner_panel_->setViewpointPathBranchSelection(index);
  // Set previous selection for combo box
  planner_panel_->setViewpointPathSelectionByItemIndex(path_selection_index);
  // Show triangulated voxels
  octree_drawer_.updateRaycastVoxels(viewpoint_path.observed_voxel_map);
  octree_drawer_.setInformationRange(0, 1);
  octree_drawer_.setWeightRange(0, 1);
}

void ViewerWidget::setViewpointPathSelectionIndex(std::size_t index) {
  const bool verbose = true;

  std::unique_lock<std::mutex> lock(mutex_);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif

  makeCurrent();

  selected_viewpoint_path_entry_index_ = index;

  const size_t viewpoint_path_branch_index =
          (selected_viewpoint_path_branch_index_ != (size_t)-1) ? selected_viewpoint_path_branch_index_ : 0;
  const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index];

  const Pose& pose = viewpoint_path.entries[index].viewpoint.pose();
  std::cout << "  Position=" << pose.getWorldPosition().transpose() << std::endl;
  std::cout << "  Orientation=" << pose.quaternion() << std::endl;
  if (planner_->hasGpsTransformation()) {
    std::cout << "  GPS=" << getGpsFromPose(pose) << std::endl;
  }
//  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(viewpoint_path_copy_[index]);

  // Compute total and incremental voxel set and information of selected viewpoint
  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[index].viewpoint_index;
  const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
  const ViewpointPlanner::VoxelWithInformationSet total_voxel_set = viewpoint_entry.voxel_set;
  // TODO: Broken. Only for triangulation mode,
  //  const ViewpointPlanner::ViewpointPathComputationData& comp_data = planner_->getViewpointPathsComputationData()[viewpoint_path_branch_index];
//  ViewpointPlanner::VoxelWithInformationSet total_voxel_set = viewpoint_entry.voxel_set;
//  if (!comp_data.triangulated_voxel_to_path_entries_map.empty()) {
//    for (const auto& entry : comp_data.triangulated_voxel_to_path_entries_map) {
//      auto it1 = std::find_if(total_voxel_set.begin(), total_voxel_set.end(),
//          [&](const ViewpointPlanner::VoxelWithInformation& vi) {
//        return vi.voxel == entry.first;
//      });
//      if (it1 != total_voxel_set.end()) {
//        auto it2 = std::find_if(entry.second.begin(), entry.second.end(),
//            [&](const std::pair<ViewpointPlanner::ViewpointEntryIndex, ViewpointPlanner::ViewpointEntryIndex>& p) {
//          return p.first == viewpoint_index || p.second == viewpoint_index;
//        });
//        if (it2 == entry.second.end()) {
//          total_voxel_set.erase(it1);
//        }
//      }
//    }
//  }
  ViewpointPlanner::VoxelMap accumulated_voxel_map;
  ViewpointPlanner::VoxelMap incremental_voxel_map;
  if (planner_panel_->isShowIncrementalVoxelSetChecked() || planner_panel_->isShowAccumulativeVoxelSetChecked()) {
    // Compute accumulated and incremental voxel map
    for (std::size_t i = 0; i <= index; ++i) {
      ViewpointPlanner::ViewpointEntryIndex other_viewpoint_index = viewpoint_path.entries[i].viewpoint_index;
      const ViewpointPlanner::VoxelWithInformationSet& other_voxel_set = planner_->getViewpointEntries()[other_viewpoint_index].voxel_set;
      for (const ViewpointPlanner::VoxelWithInformation& vi : other_voxel_set) {
        const FloatType observation_information = planner_->getOptions().viewpoint_information_factor * vi.information;
        const auto it = accumulated_voxel_map.find(vi.voxel);
        if (it == accumulated_voxel_map.end()) {
          accumulated_voxel_map.emplace(vi.voxel, observation_information);
          if (i == index) {
            incremental_voxel_map.emplace(vi.voxel, observation_information);
          }
        }
        else {
          const FloatType current_information = it->second;
          if (i == index) {
            const FloatType incremental_information = std::min(observation_information, 1 - current_information);
            incremental_voxel_map.emplace(vi.voxel, incremental_information);
          }
          const FloatType new_information = std::min(current_information + observation_information, FloatType(1));
          it->second = new_information;
        }
      }
    }
  }

  if (verbose) {
    // Print some information on the selected viewpoint
    std::cout << "Selected viewpoint " << viewpoint_index << std::endl;
    std::cout << "  total_voxel_set.size()=" << total_voxel_set.size() << std::endl;
    std::cout << "  incremental_voxel_set.size()=" << incremental_voxel_map.size() << std::endl;
    const std::size_t total_informative_voxel_count = std::count_if(total_voxel_set.cbegin(), total_voxel_set.cend(),
        [](const ViewpointPlanner::VoxelWithInformation& voxel) {
          return voxel.information > 0;
    });
    std::cout << "  total informative voxel count=" << total_informative_voxel_count << std::endl;
    if (planner_panel_->isShowIncrementalVoxelSetChecked() || planner_panel_->isShowAccumulativeVoxelSetChecked()) {
      const std::size_t incremental_informative_voxel_count = std::count_if(incremental_voxel_map.cbegin(), incremental_voxel_map.cend(),
          [](const typename ViewpointPlanner::VoxelMap::value_type& entry) {
            return entry.second > 0;
      });
      std::cout << "  incremental informative voxel count=" << incremental_informative_voxel_count << std::endl;
    }
    const FloatType total_voxel_information = std::accumulate(total_voxel_set.cbegin(), total_voxel_set.cend(),
        FloatType { 0 }, [](const FloatType& value, const ViewpointPlanner::VoxelWithInformation& voxel) {
          return value + voxel.information;
    });
    std::cout << "  total voxel information=" << total_voxel_information << std::endl;
    if (planner_panel_->isShowIncrementalVoxelSetChecked() || planner_panel_->isShowAccumulativeVoxelSetChecked()) {
      const FloatType incremental_voxel_information = std::accumulate(incremental_voxel_map.cbegin(), incremental_voxel_map.cend(),
          FloatType { 0 }, [](const FloatType& value, const typename ViewpointPlanner::VoxelMap::value_type& entry) {
            return value + entry.second;
      });
      std::cout << "  incremental voxel information=" << incremental_voxel_information << std::endl;
    }
    const auto order_it = std::find(viewpoint_path.order.begin(), viewpoint_path.order.end(), index);
    if (order_it != viewpoint_path.order.end()) {
      auto next_order_it = order_it + 1;
      if (next_order_it == viewpoint_path.order.end()) {
        next_order_it = viewpoint_path.order.begin();
      }
      std::cout << "Pose of path entry " << index << " = " << viewpoint_entry.viewpoint.pose() << std::endl;
      if (order_it != next_order_it) {
        std::cout << "Pose of next path entry " << *next_order_it << " = "
                  << viewpoint_path.entries[*next_order_it].viewpoint.pose() << std::endl;
        std::cout << "Motion from path entry " << index << " to " << *next_order_it << std::endl;
        const ViewpointPlanner::ViewpointMotion &motion = planner_->getViewpointMotion(viewpoint_index,
                                                                                       viewpoint_path.entries[*next_order_it].viewpoint_index);
        for (const ViewpointPlanner::ViewpointEntryIndex viewpoint_index : motion.viewpointIndices()) {
          std::cout << "  viewpoint index=" << viewpoint_index << std::endl;
        }
        for (auto se3_motion_it = motion.se3Motions().begin();
             se3_motion_it != motion.se3Motions().end(); ++se3_motion_it) {
          std::cout << "  sub motion: " << (se3_motion_it - motion.se3Motions().begin()) << std::endl;
          for (auto pose_it = se3_motion_it->poses().begin(); pose_it != se3_motion_it->poses().end(); ++pose_it) {
            std::cout << "  pose = " << *pose_it << std::endl;
          }
        }
      }
    }
  }

  if (planner_panel_->isUpdateCameraOnSelectionChecked()) {
    setCameraPose(viewpoint_entry.viewpoint.pose());
  }
  if (planner_panel_->isShowIncrementalVoxelSetChecked()) {
    octree_drawer_.updateRaycastVoxels(incremental_voxel_map);
  }
  else if (planner_panel_->isShowAccumulativeVoxelSetChecked()) {
    octree_drawer_.updateRaycastVoxels(accumulated_voxel_map);
  }
  else {
    octree_drawer_.updateRaycastVoxels(total_voxel_set);
  }
  octree_drawer_.setInformationRange(0, 1);
  octree_drawer_.setWeightRange(0, 1);

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  planner_lock.unlock();
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

void ViewerWidget::updateGL() {
  QGLViewer::updateGL();
}

void ViewerWidget::draw() {
  std::unique_lock<std::mutex> lock(mutex_);
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif

  makeCurrent();

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

  // Draw drawable objects:
  dense_points_drawer_.draw(pvm_matrix, dense_points_size_);
  poisson_mesh_drawer_.draw(pvm_matrix);
  if (options_.show_poisson_mesh_normals) {
    poisson_mesh_normal_drawer_.draw(pvm_matrix, width(), height(), 0.2);
  }
  region_of_interest_drawer_.draw(pvm_matrix, width(), height(), bbox_line_width_);
  bvh_bbox_drawer_.draw(pvm_matrix, width(), height(), bbox_line_width_);
  // Draw path before graph so that the graph is hidden
  viewpoint_motion_line_drawer_.draw(pvm_matrix, width(), height(), viewpoint_motion_line_width_);

  // Draw octree after mesh in case it is transparent
  octree_drawer_.draw(pvm_matrix, view_matrix, model_matrix);

  // Draw path before graph so that the graph is hidden
  viewpoint_path_drawer_.draw(pvm_matrix, width(), height());
  // Draw viewpoints at the end to enable their transparent viewports
  sparse_recon_drawer_.draw(pvm_matrix, width(), height());
  viewpoint_graph_drawer_.draw(pvm_matrix, width(), height());

  // Draw coordinate axes
  if (axes_drawer_.getDrawLines()) {
    glDisable(GL_DEPTH_TEST);
    const std::size_t axes_viewport_width = width() / 8;
    const std::size_t axes_viewport_height = height() / 8;
    const std::size_t axes_viewport_size = std::min(axes_viewport_width, axes_viewport_height);
    glViewport(0, 0, axes_viewport_size, axes_viewport_size);
    QMatrix4x4 mv_matrix;
    camera()->getModelViewMatrix(mv_matrix.data());
    mv_matrix.setColumn(3, QVector4D(0, 0, -5, 1));
    camera()->getProjectionMatrix(pvm_matrix.data());
    pvm_matrix.setToIdentity();
    pvm_matrix.ortho(-1.5f, 1.5f, -1.5f, 1.5f, 0.1f, 10.0f);
    pvm_matrix *= mv_matrix;
    axes_drawer_.draw(pvm_matrix, axes_viewport_size, axes_viewport_size, 5.0f);
    glViewport(0, 0, width(), height());
    glEnable(GL_DEPTH_TEST);
  }
}

std::pair<ViewerWidget::SelectableObjectType, boost::any> ViewerWidget::selectObject(const size_t x, const size_t y) {
  std::unique_lock<std::mutex> lock(mutex_);
#if WITH_OPENGL_OFFSCREEN
  std::unique_lock<std::mutex> opengl_lock = planner_->acquireOpenGLLock();
#endif

  makeCurrent();

  glClearColor(FloatType(1), FloatType(1), FloatType(1), FloatType(1));
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
//  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);

  glDisable(GL_MULTISAMPLE);

  QMatrix4x4 pvm_matrix;
  camera()->getModelViewProjectionMatrix(pvm_matrix.data());
  QMatrix4x4 view_matrix;
  camera()->getModelViewMatrix(view_matrix.data());
  QMatrix4x4 model_matrix; // Identity
  model_matrix.setToIdentity();

  const size_t selected_graph_entry_index = (size_t)-1;
  const size_t selected_path_entry_index = (size_t)-1;
  const bool use_selection_mode = true;
  selection_list_.clear();
  const size_t viewpoint_path_branch_index =
          (selected_viewpoint_path_branch_index_ != (size_t) -1) ? selected_viewpoint_path_branch_index_ : 0;
  const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index];
  selection_list_.reserve(planner_->getViewpointEntries().size() + viewpoint_path.entries.size());
  std::cout << "Uploading viewpoint graph" << std::endl;
  uploadViewpointGraphDrawerViewpointsWithoutLock(
          selected_graph_entry_index, use_selection_mode);
  std::cout << "Done" << std::endl;
  std::cout << "Uploading viewpoint path" << std::endl;
  uploadViewpointPathDrawerViewpointsWithoutLock(
          selected_path_entry_index, use_selection_mode);
  std::cout << "Done" << std::endl;
  // Draw selectable objects:
//  octree_drawer_.draw(pvm_matrix, view_matrix, model_matrix);
  viewpoint_graph_drawer_.draw(pvm_matrix, width(), height());
  viewpoint_path_drawer_.draw(pvm_matrix, width(), height());

  const Color4 color = readPixelColor(x, y);
  const size_t selection_index = colorToSelectionIndex(color);
//  BH_PRINT_VALUE(x);
//  BH_PRINT_VALUE(y);
//  BH_PRINT_VALUE(color);
//  BH_PRINT_VALUE(selection_index);

  glEnable(GL_MULTISAMPLE);

  size_t new_selected_graph_entry_index = (size_t)-1;
  size_t new_selected_path_entry_index = (size_t)-1;
  if (selection_index < selection_list_.size()) {
    const SelectableObjectType type = selection_list_[selection_index].first;
    const boost::any& value = selection_list_[selection_index].second;
//    BH_PRINT_VALUE(type);
    if (type == VIEWPOINT_GRAPH_ENTRY) {
//      BH_PRINT_VALUE(boost::any_cast<size_t>(&value));
      new_selected_graph_entry_index = boost::any_cast<size_t>(value);
    }
    else if (type == VIEWPOINT_PATH_ENTRY) {
//      BH_PRINT_VALUE(boost::any_cast<size_t>(&value));
      new_selected_path_entry_index = boost::any_cast<size_t>(value);
    }
  }
  uploadViewpointGraphDrawerViewpointsWithoutLock(new_selected_graph_entry_index);
  uploadViewpointPathDrawerViewpointsWithoutLock(new_selected_path_entry_index);

#if WITH_OPENGL_OFFSCREEN
  opengl_lock.unlock();
#endif
  lock.unlock();

  draw();

  if (selection_index < selection_list_.size()) {
    return selection_list_[selection_index];
  }
  else {
    return std::make_pair(SelectableObjectType::INVALID, boost::any());
  }
}

std::pair<size_t, ViewerWidget::Color4> ViewerWidget::addSelectableObject(
        const SelectableObjectType type, const boost::any& value) {
  const size_t index = selection_list_.size();
  const Color4 color = selectionIndexToColor(index);
  selection_list_.push_back(std::make_pair(type, value));
  return std::make_pair(index, color);
};

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
        custom_camera_.setZFar(bh::clamp(custom_camera_.zFar(), kZFarMin, kZFarMax));
      }
      else {
        custom_camera_.setZNear(custom_camera_.zNear() * (1.0 + event->delta() / 100.0 * kZNearSpeed));
        custom_camera_.setZNear(bh::clamp(custom_camera_.zNear(), kZNearMin, kZNearMax));
      }
      std::cout << "zNear: " << camera()->zNear() << ", zFar: " << camera()->zFar() << std::endl;
      std::cout << "scene center: " << camera()->sceneCenter() << std::endl;
      std::cout << "pivot point: " << camera()->pivotPoint() << std::endl;
    }
    event->accept();
    updateGL();
  }
  else if (event->modifiers() & Qt::ControlModifier) {
    if (event->delta() != 0) {
      sparse_recon_drawer_.changePointSize(event->delta());
      const float POINT_SIZE_SPEED = 0.1f;
      const float MIN_POINT_SIZE = 0.1f;
      const float MAX_POINT_SIZE = 100.0f;
      dense_points_size_ *= (1.0f + event->delta() / 100.0f * POINT_SIZE_SPEED);
      dense_points_size_ = bh::clamp(dense_points_size_, MIN_POINT_SIZE, MAX_POINT_SIZE);
    }
    event->accept();
    updateGL();
  } else if (event->modifiers() & Qt::AltModifier) {
    sparse_recon_drawer_.changeCameraSize(event->delta());
    viewpoint_graph_drawer_.changeCameraSize(event->delta());
    viewpoint_path_drawer_.changeCameraSize(event->delta());
    event->accept();
    updateGL();
  } else {
//      ChangeFocusDistance(event->delta());
    QGLViewer::wheelEvent(event);
  }
}

#pragma GCC optimize("O0")
void ViewerWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_0) {
    // Set camera roll to 0
    Pose pose = getCameraPose();
    const Vector3 viewing_direction = pose.quaternion().toRotationMatrix().col(2);
    const Vector3 up_direction = Vector3::UnitZ();
    pose.quaternion() = bh::getZLookAtQuaternion(viewing_direction, up_direction);
    setCameraPose(pose);
    event->accept();
  }
  else if (event->key() == Qt::Key_1) {
    // Set camera pitch and roll to 0
    Pose pose = getCameraPose();
    Vector3 viewing_direction = pose.quaternion().toRotationMatrix().col(2);
    viewing_direction(2) = 0;
    if (viewing_direction.norm() > 0) {
      const Vector3 up_direction = Vector3::UnitZ();
      pose.quaternion() = bh::getZLookAtQuaternion(viewing_direction, up_direction);
      setCameraPose(pose);
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_P) {
    // Take screenshot
    const std::string screenshot_dir = "screenshots/";
    const QDir currentDir(".");
    if (!currentDir.exists(QString::fromStdString(screenshot_dir))) {
      if (!currentDir.mkdir(QString::fromStdString(screenshot_dir))) {
        std::cout << "ERROR: Unable to create screenshot folder" << std::endl;
        event->accept();
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
    event->accept();
  }
  else if (event->key() == Qt::Key_C) {
    // Print current camera pose + info
    const Pose pose = getCameraPose();
    std::cout << "Camera pose = " << pose << std::endl;
    std::cout << "Axis: " << std::endl;
    std::cout << " x = " << pose.quaternion().toRotationMatrix().col(0).transpose() << std::endl;
    std::cout << " y = " << pose.quaternion().toRotationMatrix().col(1).transpose() << std::endl;
    std::cout << " z = " << pose.quaternion().toRotationMatrix().col(2).transpose() << std::endl;
    if (planner_->hasGpsTransformation()) {
      // Print GPS coordinates
      std::cout << "GPS = " << getGpsFromPose(pose) << std::endl;
    }
    // Compute yaw and pitch angle
    const Vector3 viewing_direction = pose.rotation().col(2);
    const FloatType yaw_radians = -std::atan2(viewing_direction(1), viewing_direction(0));
    const FloatType yaw = bh::radiansToDegrees(yaw_radians);
    const FloatType pitch_radians = std::atan2(viewing_direction(2), viewing_direction.topRows(2).squaredNorm());
    const FloatType pitch = bh::radiansToDegrees(pitch_radians);
    std::cout << "Yaw = " << yaw << ", pitch = " << pitch << std::endl;
    // Check if valid position
    std::cout << "Valid drone position: " << planner_->isValidObjectPosition(pose.getWorldPosition(), planner_->getDroneBoundingBox()) << std::endl;
    std::cout << "Exploration step: " << planner_->computeExplorationStep(pose.getWorldPosition()) << std::endl;
    if (planner_->getRoiBbox().isOutside(pose.getWorldPosition())) {
      std::cout << "ROI distance: " << planner_->getRoiBbox().distanceTo(pose.getWorldPosition()) << std::endl;
    }
    else {
      std::cout << "Inside ROI" << std::endl;
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_T) {
    // Capture raycast for full image
    captureRaycast();
    event->accept();
  }
  else if (event->key() == Qt::Key_R) {
    // Capture raycast for small window at image center
    if (event->modifiers() == Qt::ShiftModifier) {
      captureRaycastWindow(0, 0);
    }
    else {
      captureRaycastWindow(15, 15);
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_U) {
    // Capture raycast for small window at image left upper border
    captureRaycast(30, 60, 30, 60);
    event->accept();
  }
  else if (event->key() == Qt::Key_Q) {
    // Save viewpoint path as text file (i.e. for synthetic rendering)
    const std::string viewpoint_path_filename = QFileDialog::getSaveFileName(this, tr("Save viewpoint path"),
        QString("viewpoint_path.txt"),
        tr("Text file (*.txt);;All Files (*.*)")).toStdString();
    if (!viewpoint_path_filename.empty()) {
      onExportViewpointPathAsText(viewpoint_path_filename);
      std::cout << "Saved viewpoint path" << std::endl;
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_W) {
    // Save viewpoint path as Colmap sparse reconstruction
    const std::string sparse_reconstruction_path = QFileDialog::getExistingDirectory(
        this, tr("Save viewpoint path as sparse reconstruction"),
        QString("sparse/0"),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks).toStdString();
    if (!sparse_reconstruction_path.empty()) {
      onExportViewpointPathAsSparseReconstruction(sparse_reconstruction_path);
      std::cout << "Saved viewpoint path as sparse reconstruction" << std::endl;
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_A) {
    // Show raycast with current information
    captureRaycastWithCurrentInformation();
    event->accept();
  }
  else if (event->key() == Qt::Key_S) {
    // Show missing information on all voxels with weight > 0
    const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getBestViewpointPath();
    std::vector<ViewpointPlanner::OccupiedTreeType::ConstBBoxIntersectionResult> results = planner_->getBvhTree().intersects(planner_->getPositiveWeightBbox());
    ViewpointPlanner::VoxelMap voxel_map;
    for (const ViewpointPlanner::OccupiedTreeType::ConstBBoxIntersectionResult& result : results) {
      const FloatType voxel_weight = result.node->getObject()->weight;
      voxel_map.emplace(result.node, voxel_weight);
    }
    for (const ViewpointPlanner::VoxelMap::value_type& entry : viewpoint_path.observed_voxel_map) {
      const auto it = voxel_map.find(entry.first);
      if (it != voxel_map.end()) {
        it->second -= entry.second;
      }
    }
    octree_drawer_.updateRaycastVoxels(voxel_map);
    octree_drawer_.setInformationRange(0, 1);
    octree_drawer_.setWeightRange(0, 1);
    update();
    event->accept();
  }
  else if (event->key() == Qt::Key_M) {
    // Dump poisson mesh rendering from current camera pose to file
    planner_thread_.requestPoissonMeshDump(getCameraPose());
    event->accept();
  }
  else if (event->key() == Qt::Key_V) {
    // Raycast into image center
    captureInformationVoxelCenter();
    event->accept();
  }
  else if (event->key() == Qt::Key_G) {
    // Augment viewpoint path
    planner_->augmentedViewpointPathWithSparseMatchingViewpoints(&planner_->getBestViewpointPath());
    showViewpointGraph();
    showViewpointPath();
    update();
    event->accept();
  }
  else if (event->key() == Qt::Key_H) {
    // Augment viewpoint path
    planner_thread_.requestMakeViewpointPathsSparseMatchable();
    event->accept();
  }
  else if (event->key() == Qt::Key_J) {
    runInPlannerThreadAndWait(
            [&]() {
              if (!boost::filesystem::is_directory("dump")) {
                boost::filesystem::create_directories("dump");
              }
              for (const auto& image_entry : planner_->getReconstruction()->getImages()) {
                std::cout << "Drawing overlayed image: " << image_entry.second.name() << std::endl;
                const Pose& pose = image_entry.second.pose();
                const reconstruction::PinholeCameraColmap& camera = planner_->getReconstruction()->getCameras().at(image_entry.second.camera_id());
                const Viewpoint viewpoint(&camera, pose);
                planner_->getOffscreenRenderer().setCamera(camera);
                const QImage render_image = planner_->getOffscreenRenderer().drawPoissonMesh(viewpoint);
                const std::string render_output_filename = "dump/render_" + image_entry.second.name() + ".png";
                render_image.save(QString::fromStdString(render_output_filename));
                QImage real_image;
                const std::string real_image_filename = bh::joinPaths(options_.images_path, image_entry.second.name());
                if (!real_image.load(QString::fromStdString(real_image_filename))) {
                  std::cout << "Could not load real image: " << real_image_filename << std::endl;
                  continue;
                }
                if (real_image.size() != render_image.size()) {
                  std::cout << "Sizes of real and rendered image do not match: "
                            << real_image.size() << " != " << render_image.size() << std::endl;
                  continue;
                }
                const std::string real_output_filename = "dump/real_" + image_entry.second.name() + ".png";
                real_image.save(QString::fromStdString(real_output_filename));
                QImage overlay_image = render_image.convertToFormat(QImage::Format_ARGB32_Premultiplied);
                bh::qt::setImageAlphaPremultiplied(&overlay_image, options_.overlay_alpha);
                QPainter painter(&overlay_image);
                painter.setCompositionMode(QPainter::CompositionMode_DestinationOver);
                painter.drawImage(overlay_image.rect(), real_image, real_image.rect());
                painter.end();
                const std::string overlay_output_filename = "dump/overlay_" + image_entry.second.name() + ".png";
                overlay_image.save(QString::fromStdString(overlay_output_filename));
              }
              planner_->getOffscreenRenderer().setCamera(planner_->getVirtualCamera());
            }
    );
    event->accept();
  }
  else if (event->key() == Qt::Key_K) {
    if (event->modifiers() == Qt::ShiftModifier) {
      const ViewpointPlanner::ViewpointEntryIndex path_entry_index = selected_viewpoint_path_entry_index_;
      if (path_entry_index != (ViewpointPlanner::ViewpointEntryIndex) -1) {
        std::cout << "Selected viewpoint path entry: " << path_entry_index << std::endl;
        runInPlannerThreadAndWait([&]() {
                                    if (event->modifiers() == Qt::ControlModifier) {
                                      const ViewpointPlanner::ViewpointEntryIndex viewpoint_index
                                              = planner_->getBestViewpointPath().entries[path_entry_index].viewpoint_index;
                                      std::cout << "Clearing motions for path entry " << path_entry_index
                                                << ", viewpoint index " << viewpoint_index << std::endl;
                                      ViewpointPlanner::ViewpointGraph::OutEdgesWrapper out_edges = planner_->getViewpointGraph().getEdges(
                                              viewpoint_index);
                                      const size_t num_edges = out_edges.size();
                                      out_edges.clear();
                                      std::cout << "Removed " << num_edges << " motions" << std::endl;
                                    }
                                    else {
                                      const bool ignore_existing_connections = true;
                                      const bool verbose = true;
                                      const bool connected = planner_->connectViewpointPathEntry(
                                              path_entry_index,
                                              &planner_->getBestViewpointPath(),
                                              ignore_existing_connections,
                                              verbose);
                                      std::cout << "Path entry connected: " << connected << std::endl;
                                    }
                                  },
                                  [&]() {
                                    showViewpointPath(selected_viewpoint_path_entry_index_);
                                  }
        );
      }
      else {
        std::cout << "No viewpoint path entry selected" << std::endl;
      }
    }
    else {
      const ViewpointPlanner::ViewpointEntryIndex from_index = selected_viewpoint_graph_entry_index_;
      if (from_index != (ViewpointPlanner::ViewpointEntryIndex) -1) {
        std::cout << "Selected viewpoint: " << from_index << std::endl;
        runInPlannerThreadAndWait([&]() {
                                    if (event->modifiers() == Qt::ControlModifier) {
                                      std::cout << "Clearing motions for viewpoint index " << from_index << std::endl;
                                      ViewpointPlanner::ViewpointGraph::OutEdgesWrapper out_edges = planner_->getViewpointGraph().getEdges(
                                              from_index);
                                      const size_t num_edges = out_edges.size();
                                      out_edges.clear();
                                      std::cout << "Removed " << num_edges << " motions" << std::endl;
                                    }
                                    else {
                                      const bool verbose = true;
                                      const size_t num_motions = planner_->computeViewpointMotions(from_index, verbose);
                                      std::cout << "Computed " << num_motions << " viewpoint motions from viewpoint " << from_index
                                                << std::endl;
                                    }
                                  }, [&]() {
                                    showViewpointGraph(selected_viewpoint_graph_entry_index_);
                                  }
        );
      }
      else {
        std::cout << "No viewpoint graph entry selected" << std::endl;
      }
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_L) {
    bool ok;
    ViewpointPlanner::ViewpointEntryIndex initial_from_index = selected_viewpoint_graph_entry_index_;
    if (initial_from_index == (ViewpointPlanner::ViewpointEntryIndex)-1) {
      initial_from_index = 0;
    }
    const ViewpointPlanner::ViewpointEntryIndex from_index
            = QInputDialog::getInt(this, tr("Enter to viewpoint index"), tr("Viewpoint index from"),
                                   initial_from_index, 0, planner_->getViewpointEntries().size(), 1, &ok);
    if (!ok) {
      event->accept();
      return;
    }
    ViewpointPlanner::ViewpointEntryIndex initial_to_index = initial_from_index + 1;
    if (initial_to_index >= planner_->getViewpointEntries().size()) {
      initial_to_index = initial_from_index - 1;
    }
    const ViewpointPlanner::ViewpointEntryIndex to_index
            = QInputDialog::getInt(this, tr("Enter from viewpoint index"), tr("Viewpoint index to"),
                                   initial_to_index, 0, planner_->getViewpointEntries().size(), 1, &ok);
    if (!ok || from_index == to_index) {
      event->accept();
      return;
    }
    const Pose& from_pose = planner_->getViewpointEntries()[from_index].viewpoint.pose();
    const Pose& to_pose = planner_->getViewpointEntries()[to_index].viewpoint.pose();
    ViewpointPlanner::SE3Motion straight_motion;
    bool straight_motion_found;
    std::tie(straight_motion, straight_motion_found) = planner_->getMotionPlanner().findMotionStraight(from_pose, to_pose);
    if (straight_motion_found) {
      std::cout << "Found straight motion from " << from_index << " to " << to_index
                << ": distance=" << straight_motion.distance() << ", poses.size()=" << straight_motion.poses().size() << std::endl;
      ViewpointPlanner::ViewpointMotion viewpoint_motion(
              { from_index, to_index },
              { straight_motion });
      planner_->addViewpointMotion(std::move(viewpoint_motion));
      showViewpointGraph(selected_viewpoint_graph_entry_index_);
    }
    else {
      std::cout << "No straight motion from " << from_index << " to " << to_index << std::endl;
    }
    ViewpointPlanner::SE3Motion motion;
    bool motion_found;
    std::tie(motion, motion_found) = planner_->getMotionPlanner().findMotion(from_pose, to_pose);
    if (motion_found) {
      std::cout << "Found motion from " << from_index << " to " << to_index
                << ": distance=" << motion.distance() << ", poses.size()=" << motion.poses().size() << std::endl;
      if (!straight_motion_found) {
        ViewpointPlanner::ViewpointMotion viewpoint_motion(
                {from_index, to_index},
                {motion});
        planner_->addViewpointMotion(std::move(viewpoint_motion));
        showViewpointGraph(selected_viewpoint_graph_entry_index_);
      }
    }
    else {
      std::cout << "No motion from " << from_index << " to " << to_index << std::endl;
    };
    event->accept();
  }
  else if (event->key() == Qt::Key_N) {
    // Compute sparse matching between viewpoints
    if (!camera_pose_selection_valid_ || event->modifiers() == Qt::ShiftModifier) {
      selected_camera_pose_ = getCameraPose();
      camera_pose_selection_valid_ = true;
      std::cout << "Selected camera pose: " << selected_camera_pose_ << std::endl;
    }
    else {
      const Pose current_camera_pose = getCameraPose();
      planner_thread_.requestMatchCameraPoses(selected_camera_pose_, current_camera_pose);
    }
    event->accept();
  }
  else if (event->key() == Qt::Key_O) {
    // Compute visible sparse points at current camera pose
    const Viewpoint viewpoint = planner_->getVirtualViewpoint(getCameraPose());
    std::size_t num_points_visible = 0;
    std::size_t num_points_visible2 = 0;
    std::size_t num_points = 0;
    std::size_t num_points_before_camera = 0;
    for (const auto& entry : planner_->getPlannerData().getReconstruction().getPoints3D()) {
      const Point3D& point3d = entry.second;
      const Vector3 point3d_camera = viewpoint.projectWorldPointIntoCamera(point3d.getPosition());
      //      bool behind_camera;
      const Eigen::Vector2i point2d = viewpoint.camera().projectPoint(point3d_camera).cast<int>();
      const bool behind_camera = point3d_camera(2) < 0;
//      const Vector2 point2d = viewpoint.projectWorldPointIntoImage(point3d.getPosition(), &behind_camera);
      const bool visible_in_image = viewpoint.camera().isPointInViewport(point2d);
      if (visible_in_image) {
        ++num_points;
        if (!behind_camera) {
          ++num_points_before_camera;
          const FloatType point_depth = point3d_camera(2);
          BH_ASSERT(point_depth >= 0);
          const FloatType view_depth = planner_->getOffscreenRenderer().computePoissonMeshDepth(viewpoint, point2d(0), point2d(1));
          const bool visible = point_depth <= view_depth + planner_->getOptions().sparse_matching_depth_tolerance;
          if (visible) {
            ++num_points_visible2;
          }
        }
      }
      if (planner_->isSparsePointVisible(viewpoint, point3d)) {
        ++num_points_visible;
      }
    }
    std::unordered_map<Point3DId, Vector3> point_set = planner_->computeVisibleSparsePoints(
        viewpoint,
        planner_->getReconstruction()->getPoints3D().begin(),
        planner_->getReconstruction()->getPoints3D().end());
    const std::size_t num_points_visible3 = point_set.size();
    std::cout << "Visible 3D points = " << num_points_visible << std::endl;
    std::cout << "Visible 3D points 2 = " << num_points_visible2 << std::endl;
    std::cout << "Visible 3D points 3 = " << num_points_visible3 << std::endl;
    std::cout << "3D points before camera = " << num_points_before_camera << std::endl;
    std::cout << "Projecting 3D points = " << num_points << std::endl;
    event->accept();
  }
  else if (event->key() == Qt::Key_B) {
    // Add manual viewpoint entry to graph
    Pose pose = getCameraPose();
    bool no_raycast = false;
    if (event->modifiers() == Qt::ShiftModifier) {
      no_raycast = true;
    }
    if (event->modifiers() == Qt::ControlModifier) {
      pose.quaternion() = planner_->sampleBiasedOrientation(pose.getWorldPosition(), planner_->getRoiBbox());
    }
    const ViewpointPlanner::ViewpointEntryIndex new_viewpoint_index
            = planner_->addViewpointEntry(pose, no_raycast);
    std::cout << "Added viewpoint " << new_viewpoint_index << " to graph" << std::endl;
    updateViewpoints();
    event->accept();
  }
  else if (event->key() == Qt::Key_Z) {
    // Add manual viewpoint path entry
    const size_t viewpoint_path_index =
            (selected_viewpoint_path_branch_index_ != (size_t)-1) ? selected_viewpoint_path_branch_index_ : 0;
    try {
      if (event->modifiers() == Qt::ShiftModifier) {
        planner_->addViewpointPathEntryWithStereoPair(viewpoint_path_index, getCameraPose());
      }
      else {
        planner_->addViewpointPathEntry(viewpoint_path_index, getCameraPose());
      }
    }
      // TODO: Should be a exception for raycast
    catch (const bh::Error& err) {
      std::cout << "Raycast failed: " << err.what() << std::endl;
    }
    updateViewpoints();
    event->accept();
  }
  else if (event->key() == Qt::Key_X) {
    // Add manual viewpoint path entry from graph
    if (selected_viewpoint_graph_entry_index_ != (size_t) -1) {
      const size_t viewpoint_path_branch_index =
              (selected_viewpoint_path_branch_index_ != (size_t) -1) ? selected_viewpoint_path_branch_index_ : 0;
      try {
        if (event->modifiers() == Qt::ShiftModifier) {
          planner_->addViewpointPathEntryWithStereoPair(viewpoint_path_branch_index, selected_viewpoint_graph_entry_index_);
        }
        else {
          planner_->addViewpointPathEntry(viewpoint_path_branch_index, selected_viewpoint_graph_entry_index_);
        }
      }
      // TODO: Should be a exception for raycast
      catch (const bh::Error &err) {
        std::cout << "Raycast failed: " << err.what() << std::endl;
      }
      updateViewpoints();
    }
    event->accept();
  }
  else {
    QGLViewer::keyPressEvent(event);
  }
}

void ViewerWidget::mousePressEvent(QMouseEvent* event) {
  std::cout << "Mouse press event" << std::endl;
  if (event->button() == Qt::LeftButton && event->modifiers() == Qt::ShiftModifier) {
    if (selection_timer_.isActive()) {
      std::cout << "Timer is active" << std::endl;
      const size_t x = event->pos().x();
      const size_t y = event->pos().y();
      SelectableObjectType selection_type;
      boost::any selection_value;
      std::tie(selection_type, selection_value) = selectObject(x, y);
      if (selection_type == VIEWPOINT_GRAPH_ENTRY) {
        const size_t graph_entry_index = boost::any_cast<size_t>(selection_value);
        std::cout << "Selected graph entry: " << boost::any_cast<size_t>(graph_entry_index) << std::endl;
        selected_viewpoint_graph_entry_index_ = graph_entry_index;
      }
      else if (selection_type == VIEWPOINT_PATH_ENTRY) {
        const size_t path_entry_index = boost::any_cast<size_t>(selection_value);
        std::cout << "Selected path entry: " << boost::any_cast<size_t>(path_entry_index) << std::endl;
        selected_viewpoint_path_entry_index_ = path_entry_index;
      }
      update();
    }
    else {
      std::cout << "Starting timer" << std::endl;
      selection_timer_.setSingleShot(true);
      selection_timer_.start(kSelectionClickTimeMs);
    }
    event->accept();
  }
  else {
    QGLViewer::mousePressEvent(event);
  }
}

void ViewerWidget::mouseReleaseEvent(QMouseEvent* event) {
  std::cout << "Mouse release event" << std::endl;
  if (event->button() == Qt::LeftButton && event->modifiers() == Qt::ShiftModifier) {
    event->accept();
  }
  else {
    QGLViewer::mouseReleaseEvent(event);
  }
}

void ViewerWidget::mouseMoveEvent(QMouseEvent* event) {
  QGLViewer::mouseMoveEvent(event);
}

ViewerWidget::GpsCoordinateType ViewerWidget::getGpsFromPosition(const Vector3& position) const {
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = bh::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = planner_->getReconstruction()->sfmGpsTransformation().gps_reference;
  std::cout << "GPS reference: " << gps_reference << std::endl;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);
  const Vector3 enu = position;
  GpsCoordinateType gps = gps_converter.convertEnuToGps(enu.cast<GpsFloatType>());
  return gps;
}

ViewerWidget::GpsCoordinateType ViewerWidget::getGpsFromPose(const Pose& pose) const {
  return getGpsFromPosition(pose.getWorldPosition());
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
    planner_panel_->setSolveViewpointTSPText("Solving");
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
  showViewpointGraph();
  showViewpointPath();
}

void ViewerWidget::onSaveViewpointGraph(const std::string& filename) {
  runInPlannerThreadAndWait([&]() {
    planner_->saveViewpointGraph(filename);
  });
}

void ViewerWidget::onLoadViewpointGraph(const std::string& filename) {
  runInPlannerThreadAndWait([&]() {
                       planner_->loadViewpointGraph(filename);
                     },
                     [&]() {
                       showViewpointGraph();
                       showViewpointPath();
                     });
}

void ViewerWidget::onSaveViewpointPath(const std::string& filename) {
  runInPlannerThreadAndWait([&]() {
    planner_->saveViewpointPath(filename);
  });
}

void ViewerWidget::onLoadViewpointPath(const std::string& filename) {
  runInPlannerThreadAndWait([&]() {
                              planner_->loadViewpointPath(filename);
                     },
                     [&]() {
                       showViewpointGraph();
                       showViewpointPath();
                     });
}

void ViewerWidget::onExportViewpointPathAsJson(const std::string& filename) {
  ViewpointPlanner::ViewpointPath* viewpoint_path;
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    viewpoint_path = &planner_->getViewpointPaths()[selected_viewpoint_path_branch_index_];
  }
  else {
    viewpoint_path = &planner_->getBestViewpointPath();
  }
//  std::cout << "Augmenting viewpoint path with viewpoints for sparse matching" << std::endl;
//  planner_->augmentedViewpointPathForSparseMatching(viewpoint_path);
//  std::cout << "Done" << std::endl;
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    std::cout << "Exporting viewpoint path " << selected_viewpoint_path_branch_index_ << std::endl;
  }
  else {
    std::cout << "Exporting best viewpoint path" << std::endl;
  }
  planner_->exportViewpointPathAsJson(filename, *viewpoint_path);
}

void ViewerWidget::onExportViewpointPathAsText(const std::string& filename) {
  ViewpointPlanner::ViewpointPath* viewpoint_path;
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    viewpoint_path = &planner_->getViewpointPaths()[selected_viewpoint_path_branch_index_];
  }
  else {
    viewpoint_path = &planner_->getBestViewpointPath();
  }
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    std::cout << "Exporting viewpoint path " << selected_viewpoint_path_branch_index_ << std::endl;
  }
  else {
    std::cout << "Exporting best viewpoint path" << std::endl;
  }
  planner_->exportViewpointPathAsText(filename, *viewpoint_path);
}

void ViewerWidget::onExportViewpointPathAsSparseReconstruction(const std::string& path) {
  ViewpointPlanner::ViewpointPath* viewpoint_path;
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    viewpoint_path = &planner_->getViewpointPaths()[selected_viewpoint_path_branch_index_];
  }
  else {
    viewpoint_path = &planner_->getBestViewpointPath();
  }
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    std::cout << "Exporting viewpoint path as sparse reconstruction " << selected_viewpoint_path_branch_index_ << std::endl;
  }
  else {
    std::cout << "Exporting best viewpoint path" << std::endl;
  }
  planner_->exportViewpointPathAsSparseReconstruction(path, *viewpoint_path);
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
  planner_->setViewpointPathTimeConstraint(alpha);
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
  if (web_socket_server_ == nullptr || planner_->getViewpointPaths().empty()) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();

  const ViewpointPlanner::ViewpointPath* viewpoint_path;
  if (selected_viewpoint_path_branch_index_ != (std::size_t)-1 && selected_viewpoint_path_branch_index_ < planner_->getViewpointPaths().size()) {
    std::cout << "Exporting viewpoint path " << selected_viewpoint_path_branch_index_ << std::endl;
    viewpoint_path = &planner_->getViewpointPaths()[selected_viewpoint_path_branch_index_];
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
  if (web_socket_server_ == nullptr || !planner_->areViewpointPathsInitialized() || !planner_->hasGpsTransformation()) {
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
: bh::PausableThread(), planner_(planner), viewer_widget_(viewer_widget),
  operation_(Operation::NOP),
  alpha_(0), beta_(0), viewpoint_path_branch_index_(0),
  raycast_x_start_(0), raycast_x_end_(0),
  raycast_y_start_(0), raycast_y_end_(0) {
  setPausedCallback([viewer_widget]() {
    viewer_widget->signalPlannerThreadPaused();
  });
}

void ViewpointPlannerThread::requestRaycast(const Viewpoint& viewpoint,
                                            const std::size_t x_start, const std::size_t x_end,
                                            const std::size_t y_start, const std::size_t y_end) {
  raycast_viewpoint_ = viewpoint;
  raycast_x_start_ = x_start;
  raycast_x_end_ = x_end;
  raycast_y_start_ = y_start;
  raycast_y_end_ = y_end;
  setOperation(Operation::RAYCAST);
  signalContinue();
}

void ViewpointPlannerThread::customRequest(const std::function<void()>& function) {
  custom_request_barrier_ = std::promise<void>();
  custom_request_barrier_future_ = custom_request_barrier_.get_future();
  custom_request_function_ = function;
  setOperation(Operation::CUSTOM_REQUEST);
  signalContinue();
}

void ViewpointPlannerThread::waitForCustomRequest() {
  custom_request_barrier_future_.wait();
}

const Viewpoint& ViewpointPlannerThread::getRaycastViewpoint() const {
  return raycast_viewpoint_;
}

std::size_t ViewpointPlannerThread::getRaycastXStart() const {
  return raycast_x_start_;
}

std::size_t ViewpointPlannerThread::getRaycastXEnd() const {
  return raycast_x_end_;
}

std::size_t ViewpointPlannerThread::getRaycastYStart() const {
  return raycast_y_start_;
}

std::size_t ViewpointPlannerThread::getRaycastYEnd() const {
  return raycast_y_end_;
}

const std::pair<ViewpointPlanner::VoxelWithInformationSet, float>& ViewpointPlannerThread::getRaycastResults() const {
  return raycast_results_;
}

std::unordered_map<ViewpointPlanner::VoxelWrapper, ViewpointPlannerThread::Vector3, ViewpointPlanner::VoxelWrapper::Hash>
ViewpointPlannerThread::getRaycastPoissonMeshNormals() const {
  return raycast_poisson_mesh_normals_;
}

std::unordered_map<ViewpointPlanner::VoxelWrapper, float, ViewpointPlanner::VoxelWrapper::Hash>
ViewpointPlannerThread::getRaycastPoissonMeshDepth() const {
  return raycast_poisson_mesh_depth_;
}

std::unordered_map<ViewpointPlanner::VoxelWrapper, ViewpointPlannerThread::Vector2, ViewpointPlanner::VoxelWrapper::Hash>
ViewpointPlannerThread::getRaycastScreenCoordinates() const {
  return raycast_screen_coordinates_;
}

void ViewpointPlannerThread::requestPoissonMeshDump(const bh::Pose<float>& pose) {
  dump_pose_ = pose;
  setOperation(Operation::DUMP_POISSON_MESH);
  signalContinue();
}

void ViewpointPlannerThread::requestMakeViewpointPathsSparseMatchable() {
  setOperation(Operation::MAKE_VIEWPOINT_PATHS_SPARSE_MATCHABLE);
  signalContinue();
}

void ViewpointPlannerThread::requestMatchCameraPoses(const bh::Pose<float>& pose1, const bh::Pose<float>& pose2) {
  match_pose1_ = pose2;
  match_pose2_ = pose1;
  setOperation(Operation::MATCH_CAMERA_POSES);
  signalContinue();
}

ViewpointPlannerThread::Result ViewpointPlannerThread::runIteration() {
  std::unique_lock<std::mutex> lock(mutex_);

  if (operation_ == VIEWPOINT_GRAPH) {
    std::cout << "Running iterations of generateNextViewpoint()" << std::endl;
    for (std::size_t i = 0; i < 1; ++i) {
      // TODO
//      bool result = planner_->generateNextViewpointEntry();
      bool result = planner_->generateNextViewpointEntry2();
      std::cout << "Result[" << i << "] -> " << result << std::endl;
    }
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
    ViewpointPlanner::NextViewpointPathEntryStatus result = planner_->findNextViewpointPathEntries(alpha_, beta_);
    std::cout << "Result -> " << result << std::endl;
    viewer_widget_->signalViewpointsChanged();
    if (result == ViewpointPlanner::NO_IMPROVEMENT_IN_OBJECTIVE
        || result == ViewpointPlanner::NO_VIEWPOINTS_LEFT) {
      return Result::PAUSE;
    }
    else if (result == ViewpointPlanner::TIME_CONSTRAINT_EXCEEDED) {
      std::cout << "NOTE: Time constraint exceeded" << std::endl;
      return Result::PAUSE;
    }
    else {
      return Result::CONTINUE;
    }
  }
  else if (operation_ == VIEWPOINT_PATH_TSP) {
    std::cout << "Solving TSP problem for viewpoint paths" << std::endl;
    planner_->computeViewpointTour();
    std::cout << "Done" << std::endl;
    viewer_widget_->signalViewpointsChanged();
    return Result::PAUSE;
  }
  else if (operation_ == VIEWPOINT_UPDATE) {
    std::cout << "Updating display of viewpoints" << std::endl;
    viewer_widget_->signalViewpointsChanged();
    return Result::PAUSE;
  }
  else if (operation_ == RAYCAST) {
    std::cout << "Performing raycast" << std::endl;
//#if WITH_CUDA
//  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results = planner_->getRaycastHitVoxelsCuda(viewpoint);
//#else
//  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results = planner_->getRaycastHitVoxels(viewpoint);
//#endif
    std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates> tmp_raycast_results;
#if WITH_CUDA
    tmp_raycast_results =
        planner_->getRaycastHitVoxelsWithScreenCoordinates(
            raycast_viewpoint_, raycast_x_start_, raycast_x_end_, raycast_y_start_, raycast_y_end_);
#else
    tmp_raycast_results =
        planner_->getRaycastHitVoxelsWithScreenCoordinates(
            raycast_viewpoint_, raycast_x_start_, raycast_x_end_, raycast_y_start_, raycast_y_end_);
#endif
    raycast_results_.first.clear();
    raycast_results_.second = 0;
    for (const auto& result : tmp_raycast_results) {
      const float information = planner_->computeViewpointObservationScore(
          raycast_viewpoint_, result.intersection_result.node, result.screen_coordinates);
      raycast_results_.first.emplace(result.intersection_result.node, information);
      raycast_results_.second += information;
      const Vector3 normal = planner_->getOffscreenRenderer().computePoissonMeshNormalVector(
          raycast_viewpoint_, result.screen_coordinates(0), result.screen_coordinates(1));
      raycast_poisson_mesh_normals_.emplace(result.intersection_result.node, normal);
      const float depth = planner_->getOffscreenRenderer().computePoissonMeshDepth(
          raycast_viewpoint_, result.screen_coordinates(0), result.screen_coordinates(1));
      raycast_poisson_mesh_depth_.emplace(result.intersection_result.node, depth);
      raycast_screen_coordinates_.emplace(result.intersection_result.node, result.screen_coordinates);
    }
//        planner_->getRaycastHitVoxelsWithInformationScore(
//            raycast_viewpoint_, raycast_x_start_, raycast_x_end_, raycast_y_start_, raycast_y_end_);
    viewer_widget_->signalRaycastFinished();
    return Result::PAUSE;
  }
  else if (operation_ == DUMP_POISSON_MESH) {
    std::cout << "Dumping poisson mesh" << std::endl;
    if (!boost::filesystem::is_directory("dump")) {
      boost::filesystem::create_directories("dump");
    }
    planner_->getOffscreenRenderer().drawPoissonMesh(dump_pose_).save("dump/poisson_mesh_dump.png");
    planner_->getOffscreenRenderer().drawPoissonMeshNormals(dump_pose_).save("dump/poisson_mesh_normals_dump.png");
    const QImage depth_image = planner_->getOffscreenRenderer().drawPoissonMeshDepth(dump_pose_);
    depth_image.save("dump/poisson_mesh_depth_dump.png");
    const QImage depth_rgb_image = planner_->getOffscreenRenderer().convertEncodedDepthImageToRGB(depth_image, 0, 100);
    depth_rgb_image.save("dump/poisson_mesh_depth_rgb_dump.png");
    const QImage indices_image = planner_->getOffscreenRenderer().drawPoissonMeshIndices(dump_pose_);
    indices_image.save("dump/poisson_mesh_indices_dump.png");
    const QImage indices_rgb_image = planner_->getOffscreenRenderer().convertEncodedIndicesImageToRGB(
            indices_image, 0, planner_->getMesh()->m_FaceIndicesVertices.size());
    indices_rgb_image.save("dump/poisson_mesh_indices_rgb_dump.png");
    planner_->dumpSparsePoints(planner_->getVirtualViewpoint(dump_pose_), "dump/sparse_points_dump.png");
    planner_->ensureOctreeDrawerIsInitialized();
    auto drawing_handle = planner_->getOffscreenOpenGL().beginDrawing();
    const QMatrix4x4 pvm_matrix = drawing_handle->getPvmMatrixFromViewpoint(planner_->getVirtualCamera(), dump_pose_);
    const QMatrix4x4 vm_matrix = drawing_handle->getVmMatrixFromPose(dump_pose_);
//    planner_->getOctreeDrawer().setOctree(
//            planner_->getOctree(),
//            planner_->getBvhBbox().getMinimum(2),
//            planner_->getBvhBbox().getMaximum(2));
    planner_->getOctreeDrawer().setOctree(planner_->getOctree());
    planner_->getOctreeDrawer().draw(pvm_matrix, vm_matrix);
    drawing_handle.getImageQt().save("dump/octree_dump.png");
    drawing_handle.finish();
    return Result::PAUSE;
  }
  else if (operation_ == MAKE_VIEWPOINT_PATHS_SPARSE_MATCHABLE) {
    std::cout << "Making viewpoint paths sparse matchable" << std::endl;
    for (ViewpointPlanner::ViewpointPath& viewpoint_path : planner_->getViewpointPaths()) {
      planner_->makeViewpointMotionsSparseMatchable(&viewpoint_path);
    }
    viewer_widget_->signalMakeViewpointMotionsSparseMatchableFinished();
    return Result::PAUSE;
  }
  else if (operation_ == MATCH_CAMERA_POSES) {
    std::cout << "Matching of pose " << match_pose1_ << " and pose " << match_pose2_ << std::endl;
    const Viewpoint viewpoint1 = planner_->getVirtualViewpoint(match_pose1_);
    const Viewpoint viewpoint2 = planner_->getVirtualViewpoint(match_pose2_);
    BH_PRINT_VALUE(planner_->isSparseMatchable(viewpoint1, viewpoint2));
    BH_PRINT_VALUE(planner_->isSparseMatchable2(viewpoint1, viewpoint2));
    BH_PRINT_VALUE(planner_->isWithinSparseMatchingLimits(viewpoint1, viewpoint2));
    viewer_widget_->signalMatchCameraPosesFinished();
    return Result::PAUSE;
  }
  else if (operation_ == CUSTOM_REQUEST) {
    std::cout << "Running custom request" << std::endl;
    custom_request_function_();
    custom_request_barrier_.set_value();
    viewer_widget_->signalCustomRequestFinished();
    return Result::PAUSE;
  }
  else {
    std::cout << "Giving back stop result" << std::endl;
    return Result::STOP;
  }
}
