//==================================================
// viewer_widget.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

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

#include <octomap/octomap.h>
#include <qglviewer.h>
#include <ait/thread.h>
#include <ait/options.h>
#include <boost/any.hpp>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>

#include "../planner/viewpoint_planner.h"
#include "../rendering/octree_drawer.h"
#include "../rendering/sparse_reconstruction_drawer.h"
#include "../rendering/viewpoint_drawer.h"
#include "viewer_settings_panel.h"
#include "viewer_planner_panel.h"
#include "../web/web_socket_server.h"

class ViewerWidget;

class ViewpointPlannerThread : public ait::PausableThread {
public:
  USE_FIXED_EIGEN_TYPES(float);

  enum Operation {
    NOP,
    VIEWPOINT_GRAPH,
    VIEWPOINT_MOTIONS,
    VIEWPOINT_PATH,
    VIEWPOINT_PATH_TSP,
    VIEWPOINT_UPDATE,
    RAYCAST,
    DUMP_POISSON_MESH,
  };

  ViewpointPlannerThread(ViewpointPlanner* planner, ViewerWidget* viewer_widget);

  void setAlpha(double alpha) {
    alpha_ = alpha;
  }

  void setBeta(double beta) {
    beta_ = beta;
  }

  void setOperation(Operation operation) {
    operation_ = operation;
  }

  Operation operation() const {
    return operation_;
  }

  void updateViewpoints();

  void requestRaycast(const Viewpoint& viewpoint,
                      const std::size_t x_start, const std::size_t x_end,
                      const std::size_t y_start, const std::size_t y_end);

  const Viewpoint& getRaycastViewpoint() const;
  std::size_t getRaycastXStart() const;
  std::size_t getRaycastXEnd() const;
  std::size_t getRaycastYStart() const;
  std::size_t getRaycastYEnd() const;

  const std::pair<ViewpointPlanner::VoxelWithInformationSet, float>& getRaycastResults() const;

  std::unordered_map<ViewpointPlanner::VoxelWrapper, Vector3, ViewpointPlanner::VoxelWrapper::Hash>
  getRaycastPoissonMeshNormals() const;

  std::unordered_map<ViewpointPlanner::VoxelWrapper, float, ViewpointPlanner::VoxelWrapper::Hash>
  getRaycastPoissonMeshDepth() const;

  std::unordered_map<ViewpointPlanner::VoxelWrapper, Vector2, ViewpointPlanner::VoxelWrapper::Hash>
  getRaycastScreenCoordinates() const;

  void requestPoissonMeshDump(const ait::Pose<float>& pose);

protected:
  Result runIteration() override;

  void updateViewpointsInternal();

private:
  std::mutex mutex_;
  ViewpointPlanner* planner_;
  ViewerWidget* viewer_widget_;
  Operation operation_;
  double alpha_;
  double beta_;
  std::size_t viewpoint_path_branch_index_;

  Viewpoint raycast_viewpoint_;
  std::size_t raycast_x_start_;
  std::size_t raycast_x_end_;
  std::size_t raycast_y_start_;
  std::size_t raycast_y_end_;

  std::pair<ViewpointPlanner::VoxelWithInformationSet, float> raycast_results_;
  std::unordered_map<ViewpointPlanner::VoxelWrapper, Vector3, ViewpointPlanner::VoxelWrapper::Hash> raycast_poisson_mesh_normals_;
  std::unordered_map<ViewpointPlanner::VoxelWrapper, float, ViewpointPlanner::VoxelWrapper::Hash> raycast_poisson_mesh_depth_;
  std::unordered_map<ViewpointPlanner::VoxelWrapper, Vector2, ViewpointPlanner::VoxelWrapper::Hash> raycast_screen_coordinates_;

  ait::Pose<float> dump_pose_;
};

class CustomCamera : public qglviewer::Camera {
public:
  CustomCamera(const qreal z_near, const qreal z_far)
  : z_near_(z_near), z_far_(z_far) {}

  void setZNear(const qreal z_near) {
    z_near_ = z_near;
  }

  void setZFar(const qreal z_far) {
    z_far_ = z_far;
  }

  qreal zNear() const override {
    return z_near_;
  }

  qreal zFar() const override {
    return z_far_;
  }

private:
  qreal z_near_;
  qreal z_far_;
};

class ViewerWidget : public QGLViewer//, protected QOpenGLFunctions_3_3_Core
{
  Q_OBJECT

  using SparseReconstruction = reconstruction::SparseReconstruction;
  using ImageId = reconstruction::ImageId;
  using ImageColmap = reconstruction::ImageColmap;

  const double kZNearSpeed = 0.2;
  const double kZNearMin = 1e-6;
  const double kZNearMax = 10;
  const double kZFarSpeed = 0.2;
  const double kZFarMin = 20;
  const double kZFarMax = 1000;

  const int kScreenshotQuality = 90;

  const int kSelectionClickTimeMs = 500;

public:

  struct Options : ait::ConfigOptions {
    Options()
    : ait::ConfigOptions("viewpoint_planner.gui", "ViewpointPlanner GUI options") {
      addOption<bool>("websocket_enable", &websocket_enable);
      addOption<uint16_t>("websocket_port", &websocket_port);
      addOption<bool>("show_poisson_mesh_normals", &show_poisson_mesh_normals);
    }

    ~Options() override {}

    // Websocket server options
    bool websocket_enable = true;
    uint16_t websocket_port = 54321;
    bool show_poisson_mesh_normals = false;
  };

  using FloatType = float;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using Pose = ait::Pose<FloatType>;
  using Color4 = typename ViewpointDrawer<FloatType>::Color4;

  static ViewerWidget* create(const Options& options, ViewpointPlanner* planner, ViewerSettingsPanel* settings_panel,
      ViewerPlannerPanel* planner_panel, QWidget *parent = nullptr) {
    QGLFormat format;
    format.setVersion(3, 3);
//    format.setProfile(QGLFormat::CoreProfile);

    format.setProfile(QGLFormat::CompatibilityProfile);
    format.setSampleBuffers(true);
    return new ViewerWidget(options, format, planner, settings_panel, planner_panel, parent);
  }

  ViewerWidget(const Options& options, const QGLFormat& format, ViewpointPlanner* planner,
      ViewerSettingsPanel* settings_panel, ViewerPlannerPanel* planner_panel, QWidget *parent = nullptr);

  ~ViewerWidget();

  void setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max);

  void showOctree(const ViewpointPlanner::OccupancyMapType* octree);
  void showDensePoints(const ViewpointPlanner::PointCloudType* dense_points);
  void showPoissonMesh(const ViewpointPlanner::MeshType* poisson_mesh);
  void showRegionOfInterest(const ViewpointPlanner::RegionType& roi);
  void showBvhBbox(const ViewpointPlanner::BoundingBoxType& bvh_bbox);
  void showViewpointGraph(const std::size_t selected_index = (std::size_t)-1);
  void uploadViewpointGraphDrawerViewpointsWithoutLock(
          const size_t selected_index,
          const bool use_selection_mode = false);
  void showViewpointGraphMotions(const std::size_t selected_index);
  void showViewpointPath(const std::size_t selected_index = (std::size_t)-1);
  void showViewpointPathMotions(const std::size_t selected_index = (std::size_t)-1);
  void uploadViewpointPathDrawerViewpointsWithoutLock(
          const size_t selected_index,
          const bool use_selection_mode = false);
  void showSparseReconstruction(const SparseReconstruction* sparse_recon);
  void resetView();


  Color4 readPixelColor(const size_t x, const size_t y) const;
  size_t colorToSelectionIndex(const Color4& color) const;
  Color4 selectionIndexToColor(const size_t index) const;

  Pose getCameraPose() const;
  void setCameraPose(const Pose& camera_pose);
  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  GpsCoordinateType getGpsFromPose(const Pose& pose) const;

  QSize sizeHint() const override;
  int heightForWidth(int w) const override;

  void updateGL() override;

public slots:
    // Settings panel slots
  void refreshTree();
  void setDrawRaycast(bool draw_raycast);
  void captureRaycast();
  void captureRaycastWithCurrentInformation();
  void captureInformationVoxelCenter();
  void captureRaycast(
      const std::size_t x_start, const std::size_t x_end, const std::size_t y_start, const std::size_t y_end);
  void captureRaycastWindow(const std::size_t width, const std::size_t height);
  void setOccupancyBinThreshold(double occupancy_bin_threshold);
  void setColorFlags(uint32_t color_flags);
  void setDrawFreeVoxels(bool draw_free_voxels);
  void setDrawAxes(bool draw_axes);
  void setVoxelAlpha(double voxel_alpha);
  void setDrawSingleBin(bool draw_single_bin);
  void setDrawOctree(bool draw_octree);
  void setDrawCameras(bool draw_cameras);
  void setDrawViewpointGraph(bool draw_viewpoint_graph);
  void setDrawViewpointMotions(bool draw_viewpoint_motions);
  void setDrawViewpointPath(bool draw_viewpoint_path);
  void setDrawSparsePoints(bool draw_sparse_points);
  void setDrawDensePoints(bool draw_dense_points);
  void setDrawRegionOfInterest(bool draw_region_of_interest);
  void setDrawBvhBbox(bool draw_bvh_bbox);
  void setDrawPoissonMesh(bool draw_poisson_mesh);
  void setUseDroneCamera(bool use_drone_camera);
  void setImagePoseIndex(ImageId image_id);
  void setViewpointPathBranchSelectionIndex(std::size_t index);
  void setViewpointPathSelectionIndex(std::size_t index);
  void setViewpointGraphSelectionIndex(std::size_t index);
  void setMinOccupancy(double min_occupancy);
  void setMaxOccupancy(double max_occupancy);
  void setMinObservations(uint32_t min_observations);
  void setMaxObservations(uint32_t max_observations);
  void setMinVoxelSize(double min_voxel_size);
  void setMaxVoxelSize(double max_voxel_size);
  void setMinWeight(double min_weight);
  void setMaxWeight(double max_weight);
  void setMinInformation(double min_information);
  void setMaxInformation(double max_information);
  void setRenderTreeDepth(std::size_t render_tree_depth);
  void setRenderObservationThreshold(std::size_t render_observation_threshold);

  // Planner panel slots
  void pauseContinueViewpointGraph();
  void pauseContinueViewpointMotions();
  void pauseContinueViewpointPath();
  void solveViewpointTSP();
  void pauseContinueOperation(ViewpointPlannerThread::Operation operation);
  void resetViewpoints();
  void resetViewpointMotions();
  void resetViewpointPath();
  void onSaveViewpointGraph(const std::string& filename);
  void onLoadViewpointGraph(const std::string& filename);
  void onSaveViewpointPath(const std::string& filename);
  void onLoadViewpointPath(const std::string& filename);
  void onExportViewpointPathAsJson(const std::string& filename);
  void onExportViewpointPathAsText(const std::string& filename);
  void onExportViewpointPathAsSparseReconstruction(const std::string& path);
  void continuePlannerThread();
  void pausePlannerThread();

  void signalViewpointsChanged();
  void signalPlannerThreadPaused();
  void setUseFixedColors(bool use_fixed_colors);
  void setAlphaParameter(double alpha);
  void setBetaParameter(double beta);
  void setMinInformationFilter(double min_information_filter);
  void setViewpointPathLineWidth(double line_width);
  void setViewpointColorMode(std::size_t color_mode);
  void setViewpointGraphComponent(int component);

  void onRaycastFinished();

  void signalRaycastFinished();


protected slots:
  void updateViewpoints();
  void onPlannerThreadPaused();
  void sendViewpointPathToWebSocketClients();
  void sendClearSelectedPositionToWebSocketClients();
  void sendSelectedPositionToWebSocketClients(const Vector3& position);

signals:
  void viewpointsChanged();
  void plannerThreadPaused();
  void raycastFinished();

protected:
  enum SelectableObjectType {
    INVALID,
    VIEWPOINT_GRAPH_ENTRY,
    VIEWPOINT_PATH_ENTRY,
  };

  void draw() override;
  void drawWithNames() override;
  void init() override;
  void initAxesDrawer();

  void postDraw() override;
  void postSelection(const QPoint&) override;

  std::pair<SelectableObjectType, boost::any> selectObject(const size_t x, const size_t y);
  std::pair<size_t, Color4> addSelectableObject(const SelectableObjectType type, const boost::any& value);

  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void saveScreenshot(const std::string& filename);

  qglviewer::Vec eigenToQglviewer(const Vector3& eig_vec) const;
  Vector3 qglviewerToEigen(const qglviewer::Vec& qgl_vec) const;
  qglviewer::Quaternion eigenToQglviewer(const Quaternion& eig_quat) const;
  Quaternion qglviewerToEigen(const qglviewer::Quaternion& qgl_quat) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend class ViewpointPlannerThread;

  enum ViewpointColorMode : std::size_t {
    Fixed = 1,
    Component = 2,
    Information = 3,
    Indexed = 4,
  };

  std::vector<std::pair<std::string, ViewerWidget::ViewpointColorMode>> getAvailableViewpointColorModes() const;

  Options options_;

  WebSocketServer* web_socket_server_;

  CustomCamera custom_camera_;
  double z_near_coefficient_;

  std::mutex mutex_;

  ViewpointPlanner* planner_;
  bool initialized_;
  ViewerSettingsPanel* settings_panel_;
  ViewerPlannerPanel* planner_panel_;
  std::vector<std::pair<SelectableObjectType, boost::any>> selection_list_;
  QTimer selection_timer_;

  size_t selected_viewpoint_graph_entry_index_;
  size_t selected_viewpoint_path_branch_index_;
  size_t selected_viewpoint_path_entry_index_;

  const ViewpointPlanner::OccupancyMapType* octree_;
  const SparseReconstruction* sparse_recon_;
  const ViewpointPlanner::PointCloudType* dense_points_;
  FloatType dense_points_size_;
  const ViewpointPlanner::MeshType* poisson_mesh_;

  FloatType aspect_ratio_;
  LineDrawer axes_drawer_;
  OcTreeDrawer octree_drawer_;
  SparseReconstructionDrawer sparce_recon_drawer_;
  PointDrawer dense_points_drawer_;
  TriangleDrawer poisson_mesh_drawer_;
  LineDrawer poisson_mesh_normal_drawer_;
  LineDrawer region_of_interest_drawer_;
  LineDrawer bvh_bbox_drawer_;
  FloatType bbox_line_width_;

  ViewpointDrawer<FloatType> viewpoint_graph_drawer_;
  ViewpointDrawer<FloatType> viewpoint_path_drawer_;
  LineDrawer viewpoint_motion_line_drawer_;
  FloatType viewpoint_motion_line_width_;
  FloatType min_information_filter_;
  ViewpointColorMode viewpoint_color_mode_;
  int viewpoint_selected_component_;

//    QTimer* process_timer_;
  ViewpointPlannerThread planner_thread_;
  std::size_t viewpoint_path_branch_index_;

  enum RaycastMode {
    DEFAULT,
    WITH_CURRENT_INFORMATION,
    INFORMATION_VOXEL_CENTER,
  };

  RaycastMode raycast_mode_;
};
