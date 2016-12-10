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
#include "sparse_reconstruction.h"

#define _TREE_MAX_DEPTH 20

class ViewerSettingsPanel : public QWidget
{
    Q_OBJECT

public:
    ViewerSettingsPanel(QWidget *parent = 0)
    : QWidget(parent) {
        ui.setupUi(this);
        connect(ui.renderTreeDepth, SIGNAL(valueChanged(int)), this, SLOT(setRenderTreeDepth(int)));
        connect(ui.occupancyThreshold, SIGNAL(currentIndexChanged(int)), this, SLOT(setOccupancyThreshold(int)));
        connect(ui.voxelAlpha, SIGNAL(valueChanged(double)), this, SLOT(setVoxelAlpha(double)));
        connect(ui.displayAxes, SIGNAL(stateChanged(int)), this, SLOT(setDisplayAxesState(int)));
        connect(ui.drawFreeVoxels, SIGNAL(stateChanged(int)), this, SLOT(setDrawFreeVoxelsState(int)));
        connect(ui.drawSingleBin, SIGNAL(stateChanged(int)), this, SLOT(setDrawSingleBinState(int)));
        connect(ui.drawOctree, SIGNAL(stateChanged(int)), this, SLOT(setDrawOctree(int)));
        connect(ui.drawCameras, SIGNAL(stateChanged(int)), this, SLOT(setDrawCameras(int)));
        connect(ui.drawSparsePoints, SIGNAL(stateChanged(int)), this, SLOT(setDrawSparsePoints(int)));
        connect(ui.refreshTree, SIGNAL(clicked(void)), this, SLOT(signalRefreshTree()));
        connect(ui.useDroneCamera, SIGNAL(stateChanged(int)), this, SLOT(setUseDroneCamera(int)));
        connect(ui.imagePose, SIGNAL(activated(int)), this, SLOT(setImagePose(int)));
    }

    ~ViewerSettingsPanel() {
    }

    void selectOccupancyThreshold(double occupancy_threshold) {
        double min_dist = std::numeric_limits<double>::infinity();
        size_t min_index = 0;
        for (size_t i = 0; i < static_cast<size_t>(ui.occupancyThreshold->count()); ++i) {
            double oc = ui.occupancyThreshold->itemData(i).toDouble();
            double dist = std::abs(occupancy_threshold - oc);
            if (dist < min_dist) {
                min_dist = dist;
                min_index = i;
            }
        }
        ui.occupancyThreshold->setCurrentIndex(min_index);
    }

    void initializeOccupancyThresholds(const std::vector<double>& occupancy_thresholds) {
        ui.occupancyThreshold->blockSignals(true);
        ui.occupancyThreshold->clear();
        for (double occupancy : occupancy_thresholds) {
            ui.occupancyThreshold->addItem(QString::number(occupancy), QVariant(occupancy));
        }
        ui.occupancyThreshold->blockSignals(false);
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

public slots:
    void setRenderTreeDepth(int render_tree_depth) {
        emit renderTreeDepthChanged(render_tree_depth);
    }

    void setOccupancyThreshold(int index) {
        bool ok;
        double occupancy_threshold = ui.occupancyThreshold->itemData(index).toDouble(&ok);
        if (!ok) {
            throw AIT_EXCEPTION("Unable to convert occupancy threshold user data to double");
        }
        emit occupancyThresholdChanged(occupancy_threshold);
    }

    void setVoxelAlpha(double voxel_alpha) {
        emit voxelAlphaChanged(voxel_alpha);
    }

    void setDisplayAxesState(int state)  {
        emit displayAxesChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawFreeVoxelsState(int state) {
        emit drawFreeVoxelsChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawSingleBinState(int state) {
        emit drawSingleBinChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawOctree(int state) {
        emit drawOctreeChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawCameras(int state) {
        emit drawCamerasChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setDrawSparsePoints(int state) {
        emit drawSparsePointsChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void signalRefreshTree(){
        emit refreshTree();
    }

    void setUseDroneCamera(int state) {
        emit useDroneCameraChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setImagePose(int index) {
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

signals:
  void renderTreeDepthChanged(int render_tree_depth);
  void occupancyThresholdChanged(double occupancy_threshold);
  void voxelAlphaChanged(double voxel_alpha);
  void displayAxesChanged(bool display_axes);
  void drawFreeVoxelsChanged(bool draw_free_voxels);
  void drawSingleBinChanged(bool draw_single_bin);
  void drawOctreeChanged(bool draw_octree);
  void drawCamerasChanged(bool draw_cameras);
  void drawSparsePointsChanged(bool draw_sparse_points);
  void useDroneCameraChanged(bool use_drone_camera);
  void refreshTree();
  void imagePoseChanged(ImageId image_id);


private:
    Ui::ViewerSettingsPanelClass ui;
};
