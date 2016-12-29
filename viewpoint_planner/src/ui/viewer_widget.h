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
#include "../planner/viewpoint_planner.h"
#include "../rendering/octree_drawer.h"
#include "../rendering/sparse_reconstruction_drawer.h"
#include "../rendering/viewpoint_drawer.h"
#include "viewer_settings_panel.h"
#include "viewer_planner_panel.h"

class ViewerWidget;

class ViewpointPlannerThread : public ait::PausableThread {
public:
  enum Operation {
    NOP,
    VIEWPOINT_GRAPH,
    VIEWPOINT_PATH,
  };

  ViewpointPlannerThread(ViewpointPlanner* planner, ViewerWidget* viewer_widget);

  void setOperation(Operation operation) {
    operation_ = operation;
  }

  Operation operation() const {
    return operation_;
  }

  void updateViewpoints();

protected:
  Result runIteration() override;

  void updateViewpointsInternal();

private:
  ViewpointPlanner* planner_;
  ViewerWidget* viewer_widget_;
  Operation operation_;
};

class ViewerWidget : public QGLViewer
{
  Q_OBJECT

  using SparseReconstruction = reconstruction::SparseReconstruction;
  using ImageId = reconstruction::ImageId;
  using ImageColmap = reconstruction::ImageColmap;

public:
  using FloatType = float;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using Pose = ait::Pose<FloatType>;

  static ViewerWidget* create(ViewpointPlanner* planner, ViewerSettingsPanel* settings_panel,
      ViewerPlannerPanel* planner_panel, QWidget *parent = nullptr) {
    QGLFormat format;
    format.setVersion(3, 3);
//    format.setProfile(QGLFormat::CoreProfile);

    format.setProfile(QGLFormat::CompatibilityProfile);
    format.setSampleBuffers(true);
    return new ViewerWidget(format, planner, settings_panel, planner_panel, parent);
  }

  ViewerWidget(const QGLFormat& format, ViewpointPlanner* planner,
      ViewerSettingsPanel* settings_panel, ViewerPlannerPanel* planner_panel, QWidget *parent = nullptr);

  ~ViewerWidget();

  virtual void setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max);

  void showOctree(const ViewpointPlanner::OccupancyMapType* octree);
  void showViewpointPath();
  void showViewpointGraph();
  void showSparseReconstruction(const SparseReconstruction* sparse_recon);
  void resetView();

  Pose getCameraPose() const;
  void setCameraPose(const Pose& camera_pose);

  QSize sizeHint() const override;
  int heightForWidth(int w) const override;

public slots:
    // Settings panel slots
  void refreshTree();
  void setDrawRaycast(bool draw_raycast);
  void captureRaycast();
  void setOccupancyBinThreshold(double occupancy_bin_threshold);
  void setColorFlags(uint32_t color_flags);
  void setDrawFreeVoxels(bool draw_free_voxels);
  void setDisplayAxes(bool display_axes);
  void setVoxelAlpha(double voxel_alpha);
  void setDrawSingleBin(bool draw_single_bin);
  void setDrawOctree(bool draw_octree);
  void setDrawCameras(bool draw_cameras);
  void setDrawViewpointPath(bool draw_viewpoint_path);
  void setDrawViewpointGraph(bool draw_viewpoint_graph);
  void setDrawSparsePoints(bool draw_sparse_points);
  void setUseDroneCamera(bool use_drone_camera);
  void setImagePoseIndex(ImageId image_id);
  void setViewpointPathSelectionIndex(size_t index);
  void setViewpointGraphSelectionIndex(size_t index);
  void setMinOccupancy(double min_occupancy);
  void setMaxOccupancy(double max_occupancy);
  void setMinObservations(uint32_t min_observations);
  void setMaxObservations(uint32_t max_observations);
  void setMinVoxelSize(double min_voxel_size);
  void setMaxVoxelSize(double max_voxel_size);
  void setMinWeight(double min_weight);
  void setMaxWeight(double max_weight);
  void setRenderTreeDepth(size_t render_tree_depth);
  void setRenderObservationThreshold(size_t render_observation_threshold);

  // Planner panel slots
  void pauseContinueViewpointGraph();
  void pauseContinueViewpointPath();
  void resetViewpoints();
  void resetViewpointPath();
  void continuePlannerThread();
  void pausePlannerThread();

  void signalViewpointsChanged();
  void signalPlannerThreadPaused();

protected slots:
  void updateViewpoints();
  void onPlannerThreadPaused();

signals:
  void viewpointsChanged();
  void plannerThreadPaused();

protected:
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void initAxesDrawer();

    void postDraw() override;
    void postSelection(const QPoint&) override;

    void wheelEvent(QWheelEvent* event) override;

    qglviewer::Vec eigenToQglviewer(const Vector3& eig_vec) const;
    Vector3 qglviewerToEigen(const qglviewer::Vec& qgl_vec) const;
    qglviewer::Quaternion eigenToQglviewer(const Quaternion& eig_quat) const;
    Quaternion qglviewerToEigen(const qglviewer::Quaternion& qgl_quat) const;

private:
    friend class ViewpointPlannerThread;

    ViewpointPlanner* planner_;
    bool initialized_;
    ViewerSettingsPanel* settings_panel_;
    ViewerPlannerPanel* planner_panel_;
    const ViewpointPlanner::OccupancyMapType* octree_;
    const SparseReconstruction* sparse_recon_;
    bool display_axes_;

    LineDrawer axes_drawer_;
    OcTreeDrawer octree_drawer_;
    SparseReconstructionDrawer sparce_recon_drawer_;
    ViewpointDrawer<FloatType> viewpoint_graph_drawer_;
    ViewpointDrawer<FloatType> viewpoint_path_drawer_;
    FloatType aspect_ratio_;

//    QTimer* process_timer_;
    ViewpointPlannerThread planner_thread_;
    std::vector<std::tuple<ViewpointPlanner::ViewpointEntryIndex, Pose, FloatType>> viewpoint_graph_copy_;
    std::vector<std::tuple<ViewpointPlanner::ViewpointEntryIndex, Pose, FloatType>> viewpoint_path_copy_;

};
