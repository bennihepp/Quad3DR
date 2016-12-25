//==================================================
// viewpoint_planner_window.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================

#include <QDockWidget>
#include <ait/utilities.h>
#include "viewer_window.h"

using namespace std;

ViewerWindow::ViewerWindow(ViewpointPlanner* planner, QWidget *parent)
    : QMainWindow(parent), planner_(planner) {
    // Info panel at the right side
    info_panel_ = new ViewerInfoPanel(this);
    QDockWidget* info_dock = new QDockWidget("Info", this);
    info_dock->setWidget(info_panel_);
    this->addDockWidget(Qt::RightDockWidgetArea, info_dock);

    // Settings panel at the right side
    settings_panel_ = new ViewerSettingsPanel(this);
    QDockWidget* settings_dock = new QDockWidget("Settings", this);
    settings_dock->setWidget(settings_panel_);
    this->addDockWidget(Qt::RightDockWidgetArea, settings_dock);

    viewer_widget_ = ViewerWidget::create(planner_, settings_panel_, this);
    this->setCentralWidget(viewer_widget_);
    viewer_widget_->showOctree(planner_->getOctree());
    viewer_widget_->showSparseReconstruction(planner_->getReconstruction());

    // Update labels in info panel
    info_panel_->setVoxelSize(planner_->getOctree()->getResolution());
    info_panel_->setNumOfNodes(planner_->getOctree()->size());
    info_panel_->setNumOfLeafNodes(planner_->getOctree()->getNumLeafNodes());
    info_panel_->setTreeDepth(planner_->getOctree()->getTreeDepth());
    double extent_x, extent_y, extent_z;
    planner_->getOctree()->getMetricSize(extent_x, extent_y, extent_z);
    info_panel_->setExtent(QVector3D(extent_x, extent_y, extent_z));
    info_panel_->setNumOfImages(planner_->getReconstruction()->getImages().size());
    info_panel_->setNumOfSparsePoints(planner_->getReconstruction()->getPoints3D().size());
}

ViewerWindow::~ViewerWindow() {}
