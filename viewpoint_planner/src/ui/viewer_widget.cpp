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
#include <manipulatedCameraFrame.h>
#include <ait/utilities.h>
#include <ait/pose.h>
#include <ait/color.h>

using namespace std;

ViewerWidget::ViewerWidget(const QGLFormat& format, ViewpointPlanner* planner, ViewerSettingsPanel* settings_panel,
    ViewerPlannerPanel* planner_panel, QWidget *parent)
    : QGLViewer(format, parent), planner_(planner), initialized_(false),
      settings_panel_(settings_panel), planner_panel_(planner_panel),
      octree_(nullptr), sparse_recon_(nullptr),
      display_axes_(true), aspect_ratio_(-1),
      viewpoint_motion_line_width_(5),
      min_information_filter_(0),
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
    connect(settings_panel_, SIGNAL(displayAxesChanged(bool)), this, SLOT(setDisplayAxes(bool)));
    connect(settings_panel_, SIGNAL(drawSingleBinChanged(bool)), this, SLOT(setDrawSingleBin(bool)));
    connect(settings_panel_, SIGNAL(drawCamerasChanged(bool)), this, SLOT(setDrawCameras(bool)));
    connect(settings_panel_, SIGNAL(drawSparsePointsChanged(bool)), this, SLOT(setDrawSparsePoints(bool)));
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
    connect(settings_panel_, SIGNAL(renderTreeDepthChanged(std::size_t)), this, SLOT(setRenderTreeDepth(std::size_t)));
    connect(settings_panel_, SIGNAL(renderObservationThresholdChanged(std::size_t)), this, SLOT(setRenderObservationThreshold(std::size_t)));

    connect(planner_panel_, SIGNAL(pauseContinueViewpointGraph()), this, SLOT(pauseContinueViewpointGraph()));
    connect(planner_panel_, SIGNAL(pauseContinueViewpointMotions()), this, SLOT(pauseContinueViewpointMotions()));
    connect(planner_panel_, SIGNAL(pauseContinueViewpointPath()), this, SLOT(pauseContinueViewpointPath()));
    connect(planner_panel_, SIGNAL(resetViewpoints()), this, SLOT(resetViewpoints()));
    connect(planner_panel_, SIGNAL(resetViewpointMotions()), this, SLOT(resetViewpointMotions()));
    connect(planner_panel_, SIGNAL(resetViewpointPath()), this, SLOT(resetViewpointPath()));
    connect(planner_panel_, SIGNAL(saveViewpointGraph(const std::string&)), this, SLOT(onSaveViewpointGraph(const std::string&)));
    connect(planner_panel_, SIGNAL(loadViewpointGraph(const std::string&)), this, SLOT(onLoadViewpointGraph(const std::string&)));
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

    // Fill occupancy dropbox in settings panel
    FloatType selected_occupancy_bin_threshold = octree_drawer_.getOccupancyBinThreshold();
    settings_panel_->initializeOccupancyBinThresholds(octree_drawer_.getOccupancyBins());
    settings_panel_->selectOccupancyBinThreshold(selected_occupancy_bin_threshold);

    std::vector<std::pair<std::string, uint32_t>> color_flags_uint;
    for (const auto& entry : VoxelDrawer::getAvailableColorFlags()) {
      color_flags_uint.push_back(std::make_pair(entry.first, static_cast<uint32_t>(entry.second)));
    }
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

    connect(this, SIGNAL(viewpointsChanged()), this, SLOT(updateViewpoints()));
    connect(this, SIGNAL(plannerThreadPaused()), this, SLOT(onPlannerThreadPaused()));

    planner_thread_.setAlpha(planner_panel_->getAlphaParameter());
    planner_thread_.setBeta(planner_panel_->getBetaParameter());
    planner_thread_.setOperation(ViewpointPlannerThread::Operation::VIEWPOINT_GRAPH);
    planner_thread_.start();
    continuePlannerThread();
}

ViewerWidget::~ViewerWidget() {
  planner_thread_.finish();
}

void ViewerWidget::init() {
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

    setUseDroneCamera(settings_panel_->getUseDroneCamera());

    // background color defaults to white
    this->setBackgroundColor(QColor(255,255,255));
    this->qglClearColor(this->backgroundColor());

    initAxesDrawer();

    sparce_recon_drawer_.init();
    viewpoint_path_drawer_.init();
    viewpoint_graph_drawer_.init();
    viewpoint_motion_line_drawer_.init();
    if (octree_ != nullptr) {
        showOctree(octree_);
    }
    if (sparse_recon_ != nullptr) {
        showSparseReconstruction(sparse_recon_);
    }

    std::cout << "zNear: " << camera()->zNear() << ", zFar: " << camera()->zFar() << std::endl;
    // TODO: Hack, make this dependent on the scene
    camera()->setZNearCoefficient(0.01);
    camera()->setZClippingCoefficient(1);
}

void ViewerWidget::initAxesDrawer() {
  axes_drawer_.init();
  std::vector<OGLLineData> line_data;
  FloatType axes_length = 5;
  OGLVertexData axes_origin(5, 5, 20);
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
    double sizeX, sizeY, sizeZ;
    sizeX = sizeY = sizeZ = 0.;
    size_t memoryUsage = 0;
    size_t num_nodes = 0;

    ait::Timer timer;
    // get map bbx
    double lminX, lminY, lminZ, lmaxX, lmaxY, lmaxZ;
    octree_->getMetricMin(lminX, lminY, lminZ);
    octree_->getMetricMax(lmaxX, lmaxY, lmaxZ);
    // transform to world coords using map origin
    octomap::point3d pmin(lminX, lminY, lminZ);
    octomap::point3d pmax(lmaxX, lmaxY, lmaxZ);
    lminX = pmin.x(); lminY = pmin.y(); lminZ = pmin.z();
    lmaxX = pmax.x(); lmaxY = pmax.y(); lmaxZ = pmax.z();
    // update global bbx
    if (lminX < minX) minX = lminX;
    if (lminY < minY) minY = lminY;
    if (lminZ < minZ) minZ = lminZ;
    if (lmaxX > maxX) maxX = lmaxX;
    if (lmaxY > maxY) maxY = lmaxY;
    if (lmaxZ > maxZ) maxZ = lmaxZ;
    double lsizeX, lsizeY, lsizeZ;
    // update map stats
    octree_->getMetricSize(lsizeX, lsizeY, lsizeZ);
    if (lsizeX > sizeX) sizeX = lsizeX;
    if (lsizeY > sizeY) sizeY = lsizeY;
    if (lsizeZ > sizeZ) sizeZ = lsizeZ;
    memoryUsage += octree_->memoryUsage();
    num_nodes += octree_->size();
    timer.printTiming("Computing octree metrics");

    refreshTree();

    setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));
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

  // Compute min-max of information value
  ait::MinMaxTracker<FloatType> min_max;
  for (const auto& entry : viewpoint_graph_copy_) {
    FloatType total_information = std::get<2>(entry);
    min_max.update(total_information);
  }

  bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  bool valid_selection = selected_index != (std::size_t)-1;
  bool use_fixed_colors = planner_panel_->isUseFixedColorsChecked();
  if (inspect_graph_motions && valid_selection) {
    std::cout << "Viewpoint " << selected_index << " has "
        << planner_->getViewpointGraph().getEdges(selected_index).size() << " edges" << std::endl;
  }
  // Make a copy of the graph poses so that we can later on access
  // viewpoints select by the user
  std::vector<Pose> poses;
  std::vector<typename ViewpointDrawer<FloatType>::Color4> colors;
  const ait::ColorMapHot<FloatType> cmap;
  for (const auto& entry : viewpoint_graph_copy_) {
    FloatType total_information = std::get<2>(entry);
    if (inspect_graph_motions && valid_selection) {
      ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(entry);
      if (viewpoint_index != selected_index && planner_->getViewpointGraph().getEdges(selected_index).count(viewpoint_index) == 0) {
        continue;
      }
    }
    const Pose& pose = std::get<1>(entry);
    typename ViewpointDrawer<FloatType>::Color4 color;
    if (use_fixed_colors) {
      color = typename ViewpointDrawer<FloatType>::Color4(0.7f, 0.8f, 0.0f, 0.6f);
    }
    else {
      FloatType value = min_max.normalize(total_information);
      color = cmap.map(value, 0.6f);
    }
    poses.push_back(pose);
    colors.push_back(color);
  }
  // Draw viewpoint camera frustums
  viewpoint_graph_drawer_.setCamera(sparse_recon_->getCameras().cbegin()->second);
  viewpoint_graph_drawer_.setViewpoints(poses, colors);

  planner_panel_->setViewpointGraphSize(viewpoint_graph_copy_.size());
  lock.unlock();

  update();
}

void ViewerWidget::showViewpointGraphMotions(const std::size_t selected_index) {
  if (!initialized_) {
      return;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  // Draw motion path
  std::vector<OGLLineData> lines;
  const OGLColorData line_color(0.7f, 0.1f, 0.0f, 1.0f);
  const ViewpointPlanner::ViewpointEntryVector& viewpoint_entries = planner_->getViewpointEntries();
  const ViewpointPlanner::ViewpointGraph& viewpoint_graph = planner_->getViewpointGraph();
  for (const auto& edge : viewpoint_graph.getEdges(selected_index)) {
    const Vector3 from_pos = viewpoint_entries[selected_index].viewpoint.pose().getWorldPosition();
    const ViewpointPlanner::ViewpointEntryIndex to_viewpoint_index = edge.first;
    const Vector3 to_pos = viewpoint_entries[to_viewpoint_index].viewpoint.pose().getWorldPosition();
    const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
    const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
    lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color), OGLVertexDataRGBA(vertex2, line_color));
  }
  std::cout << "Selected viewpoint " << selected_index << " has "
      << viewpoint_graph.getEdges(selected_index).size() << " edges" << std::endl;
  viewpoint_motion_line_drawer_.upload(lines);

  lock.unlock();

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

  // Generate OGL data for viewpoint drawer
  std::vector<Pose> poses;
  ait::MinMaxTracker<FloatType> min_max;
  for (const auto& entry : viewpoint_path_copy_) {
    const Pose& pose = std::get<1>(entry);
    poses.push_back(pose);
    min_max.update(std::get<2>(entry));
  }
  viewpoint_path_drawer_.setCamera(sparse_recon_->getCameras().cbegin()->second);

  // Draw viewpoint camera frustums
  if (planner_panel_->isUseFixedColorsChecked()) {
    typename ViewpointDrawer<FloatType>::Color4 color(0.0f, 0.1f, 0.8f, 0.6f);
    viewpoint_path_drawer_.setViewpoints(poses, color);
  }
  else {
    const ait::ColorMapHot<FloatType> cmap;
    std::vector<typename ViewpointDrawer<FloatType>::Color4> colors;
    for (const auto& entry : viewpoint_path_copy_) {
      FloatType information = std::get<2>(entry);
      FloatType value = min_max.normalize(information);
      colors.push_back(cmap.map(value, 0.6f));
    }
    viewpoint_path_drawer_.setViewpoints(poses, colors);
  }

  // Draw motion path
  bool inspect_graph_motions = planner_panel_->isInspectViewpointGraphMotionsChecked();
  bool valid_selection = selected_index != (std::size_t)-1;
  std::vector<OGLLineData> lines;
  if (inspect_graph_motions && valid_selection) {
    const OGLColorData line_color(0.7f, 0.1f, 0.0f, 1.0f);
    const ViewpointPlanner::ViewpointPath viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index_];
    const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[selected_index].viewpoint_index;
    const ViewpointPlanner::ViewpointEntry viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
    std::cout << "Viewpoint " << viewpoint_index << " has "
        << planner_->getViewpointGraph().getEdges(viewpoint_index).size() << " edges" << std::endl;
    for (const auto& edge : planner_->getViewpointGraph().getEdges(viewpoint_index)) {
      const ViewpointPlanner::ViewpointEntry other_viewpoint_entry = planner_->getViewpointEntries()[edge.first];
      const Vector3 from_pos = viewpoint_entry.viewpoint.pose().getWorldPosition();
      const Vector3 to_pos = other_viewpoint_entry.viewpoint.pose().getWorldPosition();
      const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
      const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
      lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color), OGLVertexDataRGBA(vertex2, line_color));
    }
  }
  else if (viewpoint_path_copy_.size() > 1) {
    const OGLColorData line_color(0.0f, 0.1f, 0.8f, 1.0f);
    for (auto it = viewpoint_path_copy_.cbegin() + 1; it != viewpoint_path_copy_.cend(); ++it) {
      const Vector3 from_pos = std::get<1>(*(it-1)).getWorldPosition();
      const Vector3 to_pos = std::get<1>(*it).getWorldPosition();
      const OGLVertexData vertex1(from_pos(0), from_pos(1), from_pos(2));
      const OGLVertexData vertex2(to_pos(0), to_pos(1), to_pos(2));
      lines.emplace_back(OGLVertexDataRGBA(vertex1, line_color), OGLVertexDataRGBA(vertex2, line_color));
    }
  }
  viewpoint_motion_line_drawer_.upload(lines);

  planner_panel_->setViewpointPathSize(viewpoint_path_copy_.size());
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

void ViewerWidget::captureRaycast()
{
  Pose camera_pose = getCameraPose();
//  std::cout << "raycast pose: " << camera_pose << std::endl;
//  std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, FloatType>> raycast_results = planner_->getRaycastHitVoxels(camera_pose);
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results = planner_->getRaycastHitVoxelsBVH(camera_pose);
  std::vector<std::pair<ViewpointPlannerData::OccupiedTreeType::IntersectionResult, FloatType>> tmp;
  tmp.reserve(raycast_results.size());
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    FloatType information = planner_->computeInformationScore(*it);
    tmp.push_back(std::make_pair(*it, information));
  }
  octree_drawer_.updateRaycastVoxels(tmp);
  update();
}

void ViewerWidget::resetView()
{
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
//  std::cout << "set pose: " << pose << std::endl;
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

void ViewerWidget::setDisplayAxes(bool display_axes)
{
  display_axes_ = display_axes;
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
    update();
}

void ViewerWidget::setViewpointGraphSelectionIndex(const std::size_t index) {
  std::cout << "index=" << index << std::endl;
  std::unique_lock<std::mutex> lock(mutex_);
//  const Pose& pose = std::get<1>(viewpoint_graph_copy_[index]);
  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(viewpoint_graph_copy_[index]);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
  const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
  if (planner_panel_->isUpdateCameraOnSelectionChecked()) {
    setCameraPose(viewpoint_entry.viewpoint.pose());
  }
  octree_drawer_.updateRaycastVoxels(viewpoint_entry.voxel_set);
  lock.unlock();
  showViewpointGraph(index);
  showViewpointGraphMotions(index);
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
  planner_panel_->setViewpointPathSelection(path_selection_index);
}

void ViewerWidget::setViewpointPathSelectionIndex(std::size_t index) {
  std::unique_lock<std::mutex> lock(mutex_);
  //  const Pose& pose = std::get<1>(viewpoint_path_copy_[index]);
  const ViewpointPlanner::ViewpointEntryIndex viewpoint_index = std::get<0>(viewpoint_path_copy_[index]);
  std::unique_lock<std::mutex> planner_lock = planner_->acquireLock();
  const ViewpointPlanner::ViewpointEntry& viewpoint_entry = planner_->getViewpointEntries()[viewpoint_index];
  if (planner_panel_->isUpdateCameraOnSelectionChecked()) {
    setCameraPose(viewpoint_entry.viewpoint.pose());
  }
  if (planner_panel_->isShowIncrementalVoxelSetChecked()) {
    const ViewpointPlanner::ViewpointPath& viewpoint_path = planner_->getViewpointPaths()[viewpoint_path_branch_index_];
    ViewpointPlanner::ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[index].viewpoint_index;
    ViewpointPlanner::VoxelWithInformationSet voxel_set = planner_->getViewpointEntries()[viewpoint_index].voxel_set;
    for (std::size_t i = 0; i < index; ++i) {
      ViewpointPlanner::ViewpointEntryIndex other_viewpoint_index = viewpoint_path.entries[i].viewpoint_index;
      const ViewpointPlanner::VoxelWithInformationSet& other_voxel_set = planner_->getViewpointEntries()[other_viewpoint_index].voxel_set;
      for (const ViewpointPlanner::VoxelWithInformation& voxel_with_information : other_voxel_set) {
        voxel_set.erase(voxel_with_information);
      }
    }
    octree_drawer_.updateRaycastVoxels(voxel_set);
  }
  else {
    octree_drawer_.updateRaycastVoxels(viewpoint_entry.voxel_set);
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
    qglviewer::Vec min_vec(-50, -50, -20);
    qglviewer::Vec max_vec(50, 50, 50);
    QGLViewer::setSceneBoundingBox(min_vec, max_vec);
//    QGLViewer::setSceneBoundingBox(min, max);
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
  // Draw path before graph so that the graph is hidden
  viewpoint_path_drawer_.draw(pvm_matrix, width(), height());
  viewpoint_motion_line_drawer_.draw(pvm_matrix, width(), height(), viewpoint_motion_line_width_);
  viewpoint_graph_drawer_.draw(pvm_matrix, width(), height());

  if (display_axes_) {
    axes_drawer_.draw(pvm_matrix, width(), height(), 5.0f);
  }
}

void ViewerWidget::drawWithNames() {}

void ViewerWidget::postDraw() {}

void ViewerWidget::postSelection(const QPoint&) {}

void ViewerWidget::wheelEvent(QWheelEvent* event) {
  if (event->modifiers() & Qt::ControlModifier) {
    sparce_recon_drawer_.changePointSize(event->delta());
    event->accept();
    updateGL();
  } else if (event->modifiers() & Qt::AltModifier) {
    sparce_recon_drawer_.changeCameraSize(event->delta());
    viewpoint_graph_drawer_.changeCameraSize(event->delta());
    viewpoint_path_drawer_.changeCameraSize(event->delta());
    event->accept();
    updateGL();
  } else if (event->modifiers() & Qt::ShiftModifier) {
//      ChangeNearPlane(event->delta());
    QGLViewer::wheelEvent(event);
  } else {
//      ChangeFocusDistance(event->delta());
    QGLViewer::wheelEvent(event);
  }
}

void ViewerWidget::pausePlannerThread() {
  if (planner_thread_.isRunning()) {
    planner_panel_->setPauseContinueViewpointGraphEnabled(false);
    planner_panel_->setPauseContinueViewpointMotionsEnabled(false);
    planner_panel_->setPauseContinueViewpointPathEnabled(false);
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
    planner_panel_->setPauseContinueViewpointGraphEnabled(false);
    planner_panel_->setPauseContinueViewpointMotionsEnabled(false);
    planner_panel_->setPauseContinueViewpointPathEnabled(false);
    planner_panel_->setResetViewpointsEnabled(false);
    planner_panel_->setResetViewpointMotionsEnabled(false);
    planner_panel_->setResetViewpointPathEnabled(false);
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
  planner_thread_.signalContinue();
}

void ViewerWidget::onPlannerThreadPaused() {
  planner_panel_->setPauseContinueViewpointGraphEnabled(true);
  planner_panel_->setPauseContinueViewpointGraphText("Continue");
  planner_panel_->setPauseContinueViewpointPathEnabled(true);
  planner_panel_->setPauseContinueViewpointMotionsText("Continue");
  planner_panel_->setPauseContinueViewpointMotionsEnabled(true);
  planner_panel_->setPauseContinueViewpointPathText("Continue");
  planner_panel_->setResetViewpointsEnabled(true);
  planner_panel_->setResetViewpointMotionsEnabled(true);
  planner_panel_->setResetViewpointPathEnabled(true);
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

void ViewerWidget::signalViewpointsChanged() {
  emit viewpointsChanged();
}

void ViewerWidget::signalPlannerThreadPaused() {
  emit plannerThreadPaused();
}

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

void ViewerWidget::updateViewpoints() {
  showViewpointGraph();
  showViewpointPath();
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
  for (ViewpointPlanner::ViewpointEntryIndex viewpoint_index : planner_->getViewpointGraph()) {
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
}
