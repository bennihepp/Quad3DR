//==================================================
// viewer_settings_panel.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <cmath>
#include <QWidget>
#include <ait/common.h>
#include "ui_viewer_settings_panel.h"
#include "../reconstruction/sparse_reconstruction.h"

#define _TREE_MAX_DEPTH 20

class ViewerSettingsPanel : public QWidget {
    Q_OBJECT

    using ImageId = reconstruction::ImageId;

public:
    ViewerSettingsPanel(QWidget *parent = 0)
    : QWidget(parent) {
        ui.setupUi(this);
        connect(ui.occupancyBinThreshold, SIGNAL(currentIndexChanged(int)), this, SLOT(setOccupancyBinThresholdInternal(int)));
        connect(ui.colorFlags, SIGNAL(currentIndexChanged(int)), this, SLOT(setColorFlagsInternal(int)));
        connect(ui.voxelAlpha, SIGNAL(valueChanged(double)), this, SLOT(setVoxelAlphaInternal(double)));
        connect(ui.displayAxes, SIGNAL(stateChanged(int)), this, SLOT(setDisplayAxesStateInternal(int)));
        connect(ui.drawFreeVoxels, SIGNAL(stateChanged(int)), this, SLOT(setDrawFreeVoxelsStateInternal(int)));
        connect(ui.drawSingleBin, SIGNAL(stateChanged(int)), this, SLOT(setDrawSingleBinStateInternal(int)));
        connect(ui.drawOctree, SIGNAL(stateChanged(int)), this, SLOT(setDrawOctreeInternal(int)));
        connect(ui.drawCameras, SIGNAL(stateChanged(int)), this, SLOT(setDrawCamerasInternal(int)));
        connect(ui.drawSparsePoints, SIGNAL(stateChanged(int)), this, SLOT(setDrawSparsePointsInternal(int)));
        connect(ui.refreshTree, SIGNAL(clicked(void)), this, SLOT(signalRefreshTreeInternal()));
        connect(ui.drawRaycast, SIGNAL(stateChanged(int)), this, SLOT(setDrawRaycastInternal(int)));
        connect(ui.captureRaycast, SIGNAL(clicked(void)), this, SLOT(signalCaptureRaycastInternal()));
        connect(ui.useDroneCamera, SIGNAL(stateChanged(int)), this, SLOT(setUseDroneCameraInternal(int)));
        connect(ui.imagePose, SIGNAL(activated(int)), this, SLOT(setImagePoseInternal(int)));
        connect(ui.minOccupancy, SIGNAL(valueChanged(double)), this, SLOT(setMinOccupancyInternal(double)));
        connect(ui.maxOccupancy, SIGNAL(valueChanged(double)), this, SLOT(setMaxOccupancyInternal(double)));
        connect(ui.minObservations, SIGNAL(valueChanged(int)), this, SLOT(setMinObservationsInternal(int)));
        connect(ui.maxObservations, SIGNAL(valueChanged(int)), this, SLOT(setMaxObservationsInternal(int)));
        connect(ui.minVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(setMinVoxelSizeInternal(double)));
        connect(ui.maxVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(setMaxVoxelSizeInternal(double)));
        connect(ui.minWeight, SIGNAL(valueChanged(double)), this, SLOT(setMinWeightInternal(double)));
        connect(ui.maxWeight, SIGNAL(valueChanged(double)), this, SLOT(setMaxWeightInternal(double)));
        connect(ui.renderTreeDepth, SIGNAL(valueChanged(int)), this, SLOT(setRenderTreeDepthInternal(int)));
        connect(ui.renderObservationThreshold, SIGNAL(valueChanged(int)), this, SLOT(setRenderObservationThresholdInternal(int)));
    }

    ~ViewerSettingsPanel() {
    }

    void selectOccupancyBinThreshold(double occupancy_threshold) {
      double min_dist = std::numeric_limits<double>::infinity();
      std::size_t min_index = 0;
      for (std::size_t i = 0; i < static_cast<std::size_t>(ui.occupancyBinThreshold->count()); ++i) {
        double oc = ui.occupancyBinThreshold->itemData(i).toDouble();
        double dist = std::abs(occupancy_threshold - oc);
        if (dist < min_dist) {
          min_dist = dist;
          min_index = i;
        }
      }
      ui.occupancyBinThreshold->setCurrentIndex(min_index);
    }

    void initializeOccupancyBinThresholds(const std::vector<float>& occupancy_thresholds) {
      ui.occupancyBinThreshold->blockSignals(true);
      ui.occupancyBinThreshold->clear();
      for (float occupancy : occupancy_thresholds) {
        ui.occupancyBinThreshold->addItem(QString::number(occupancy), QVariant(occupancy));
      }
      ui.occupancyBinThreshold->blockSignals(false);
    }

    void initializeImagePoses(const std::vector<std::pair<std::string, ImageId>>& entries) {
      ui.imagePose->blockSignals(true);
      ui.imagePose->clear();
      ui.imagePose->addItem(QString::fromStdString("<free view>"), QVariant(static_cast<int>(-1)));
      for (const auto& entry : entries) {
        ui.imagePose->addItem(QString::fromStdString(entry.first), QVariant(static_cast<int>(entry.second)));
      }
      ui.imagePose->blockSignals(false);
    }

    void selectColorFlags(const uint32_t color_flags) {
      for (std::size_t i = 0; i < static_cast<std::size_t>(ui.colorFlags->count()); ++i) {
        uint32_t entry_flags = ui.colorFlags->itemData(i).toUInt();
        if (color_flags == entry_flags) {
          ui.colorFlags->setCurrentIndex(i);
          return;
        }
      }
      throw AIT_EXCEPTION(std::string("Could not find corresponding color flags in combo box: ") + std::to_string(color_flags));
    }

    void initializeColorFlags(std::vector<std::pair<std::string, uint32_t>>& entries) {
      ui.colorFlags->blockSignals(true);
      ui.colorFlags->clear();
      for (const auto& entry : entries) {
          ui.colorFlags->addItem(QString::fromStdString(entry.first), QVariant(entry.second));
      }
      ui.colorFlags->blockSignals(false);
    }

    bool getUseDroneCamera() const {
      Qt::CheckState state = ui.useDroneCamera->checkState();
      return state == Qt::PartiallyChecked || state == Qt::Checked;
    }

    double getMinOccupancy() const {
      return ui.minOccupancy->value();
    }

    void setMinOccupancy(double min_occupancy) {
      ui.minOccupancy->setValue(min_occupancy);
    }

    double getMaxOccupancy() const {
      return ui.maxOccupancy->value();
    }

    void setMaxOccupancy(double max_occupancy) {
      ui.maxOccupancy->setValue(max_occupancy);
    }

    std::size_t getMinObservations() const {
      return static_cast<std::size_t>(ui.minObservations->value());
    }

    void setMinObservations(std::size_t min_observations) {
      ui.minObservations->setValue(static_cast<int>(min_observations));
    }

    std::size_t getMaxObservations() const {
      return static_cast<std::size_t>(ui.maxObservations->value());
    }

    void setMaxObservations(std::size_t max_observations) {
      ui.maxObservations->setValue(static_cast<int>(max_observations));
    }

    double getMinVoxelSize() const {
      return ui.minVoxelSize->value();
    }

    void setMinVoxelSize(double min_voxel_size) {
      ui.minVoxelSize->setValue(min_voxel_size);
    }

    double getMaxVoxelSize() const {
      return ui.maxVoxelSize->value();
    }

    void setMaxVoxelSize(double max_voxel_size) {
      ui.maxVoxelSize->setValue(max_voxel_size);
    }

    double getMinWeight() const {
      return ui.minWeight->value();
    }

    void setMinWeight(double min_weight) {
      ui.minWeight->setValue(min_weight);
    }

    double getMaxWeight() const {
      return ui.maxWeight->value();
    }

    void setMaxWeight(double max_weight) {
      ui.maxWeight->setValue(max_weight);
    }

    std::size_t getRenderTreeDepth() const {
      return ui.renderTreeDepth->value();
    }

    void setRenderTreeDepth(std::size_t tree_depth) {
      ui.renderTreeDepth->setValue(tree_depth);
    }

    std::size_t getRenderObservationThreshold() const {
      return ui.renderObservationThreshold->value();
    }

    void setRenderObservationThreshold(std::size_t observation_threshold) {
      ui.renderObservationThreshold->setValue(observation_threshold);
    }

protected slots:
    void setOccupancyBinThresholdInternal(int index) {
        bool ok;
        double occupancy_threshold = ui.occupancyBinThreshold->itemData(index).toDouble(&ok);
        if (!ok) {
            throw AIT_EXCEPTION("Unable to convert occupancy threshold user data to double");
        }
        emit occupancyBinThresholdChanged(occupancy_threshold);
    }

    void setColorFlagsInternal(int index) {
        bool ok;
        uint32_t color_flags = ui.colorFlags->itemData(index).toUInt(&ok);
        if (!ok) {
            throw AIT_EXCEPTION("Unable to convert color flags user data to uint");
        }
        emit colorFlagsChanged(color_flags);
    }

    void setVoxelAlphaInternal(double voxel_alpha) {
        emit voxelAlphaChanged(voxel_alpha);
    }

    void setDisplayAxesStateInternal(int state)  {
        emit displayAxesChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawFreeVoxelsStateInternal(int state) {
        emit drawFreeVoxelsChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawSingleBinStateInternal(int state) {
        emit drawSingleBinChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawOctreeInternal(int state) {
        emit drawOctreeChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawCamerasInternal(int state) {
        emit drawCamerasChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawSparsePointsInternal(int state) {
        emit drawSparsePointsChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void signalRefreshTreeInternal() {
        emit refreshTree();
    }

    void setDrawRaycastInternal(int state) {
      emit drawRaycastChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void signalCaptureRaycastInternal() {
      emit captureRaycast();
    }

    void setUseDroneCameraInternal(int state) {
        emit useDroneCameraChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setImagePoseInternal(int index) {
        bool ok;
        int user_data = ui.imagePose->itemData(index).toInt(&ok);
        if (!ok) {
            throw AIT_EXCEPTION("Unable to convert camera pose user data user data to int");
        }
        if (user_data >= 0) {
          ImageId image_id = static_cast<ImageId>(user_data);
            emit imagePoseChanged(image_id);
        }
    }

    void setMinOccupancyInternal(double min_occupancy) {
      emit minOccupancyChanged(min_occupancy);
    }

    void setMaxOccupancyInternal(double max_occupancy) {
      emit maxOccupancyChanged(max_occupancy);
    }

    void setMinObservationsInternal(int min_observations) {
      emit minObservationsChanged(static_cast<uint32_t>(min_observations));
    }

    void setMaxObservationsInternal(int max_observations) {
      emit maxObservationsChanged(static_cast<uint32_t>(max_observations));
    }

    void setMinVoxelSizeInternal(double min_voxel_size) {
      emit minVoxelSizeChanged(min_voxel_size);
    }

    void setMaxVoxelSizeInternal(double max_voxel_size) {
      emit maxVoxelSizeChanged(max_voxel_size);
    }

    void setMinWeightInternal(double min_weight) {
      emit minWeightChanged(min_weight);
    }

    void setMaxWeightInternal(double max_weight) {
      emit maxWeightChanged(max_weight);
    }

    void setRenderTreeDepthInternal(int render_tree_depth) {
        emit renderTreeDepthChanged(static_cast<std::size_t>(render_tree_depth));
    }

    void setRenderObservationThresholdInternal(int observation_threshold) {
      emit renderObservationThresholdChanged(static_cast<std::size_t>(observation_threshold));
    }

signals:
  void occupancyBinThresholdChanged(double occupancy_threshold);
  void colorFlagsChanged(uint32_t color_flags);
  void voxelAlphaChanged(double voxel_alpha);
  void displayAxesChanged(bool display_axes);
  void drawFreeVoxelsChanged(bool draw_free_voxels);
  void drawSingleBinChanged(bool draw_single_bin);
  void drawOctreeChanged(bool draw_octree);
  void drawCamerasChanged(bool draw_cameras);
  void drawSparsePointsChanged(bool draw_sparse_points);
  void useDroneCameraChanged(bool use_drone_camera);
  void refreshTree();
  void drawRaycastChanged(bool draw_raycast);
  void captureRaycast();
  void imagePoseChanged(ImageId image_id);
  void minOccupancyChanged(double min_occupancy);
  void maxOccupancyChanged(double max_occupancy);
  void minObservationsChanged(uint32_t min_observations);
  void maxObservationsChanged(uint32_t max_observations);
  void minVoxelSizeChanged(double min_voxel_size);
  void maxVoxelSizeChanged(double max_voxel_size);
  void minWeightChanged(double min_weight);
  void maxWeightChanged(double max_weight);
  void renderTreeDepthChanged(std::size_t render_tree_depth);
  void renderObservationThresholdChanged(std::size_t render_tree_depth);


private:
    Ui::ViewerSettingsPanelClass ui;
};
