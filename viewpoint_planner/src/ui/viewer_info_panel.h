//==================================================
// viewer_info_panel.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <cmath>
#include <QWidget>
#include <QVector3D>
#include "ui_viewer_info_panel.h"

#define _TREE_MAX_DEPTH 20

class ViewerInfoPanel : public QWidget
{
    Q_OBJECT

public:
    ViewerInfoPanel(QWidget *parent = 0)
    : QWidget(parent) {
        ui.setupUi(this);
    }

    ~ViewerInfoPanel() {
    }

public slots:
    void setNumOfNodes(size_t num_of_nodes) {
        ui.numOfNodes->setText(QString::number(num_of_nodes));
    }

    void setNumOfLeafNodes(size_t num_of_leaf_nodes) {
        ui.numOfLeafNodes->setText(QString::number(num_of_leaf_nodes));
    }

    void setVoxelSize(double voxel_size) {
        ui.voxelSize->setText(QString::number(voxel_size));
    }

    void setTreeDepth(size_t tree_depth) {
        ui.treeDepth->setText(QString::number(tree_depth));
    }

    void setExtent(QVector3D extent) {
        ui.extent->setText(QString::number(extent.x()) + ", " + QString::number(extent.y()) + ", " + QString::number(extent.z()));
    }

    void setNumOfImages(size_t num_of_images) {
        ui.numOfImages->setText(QString::number(num_of_images));
    }

    void setNumOfSparsePoints(size_t num_of_sparse_points) {
        ui.numOfSparsePoints->setText(QString::number(num_of_sparse_points));
    }

private:
    Ui::ViewerInfoPanelClass ui;
};
