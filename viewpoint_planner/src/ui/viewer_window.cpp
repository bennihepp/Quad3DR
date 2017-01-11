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
    : QMainWindow(parent), planner_(planner), viewer_widget_(nullptr) {
    // Info panel at the right side
    info_panel_ = new ViewerInfoPanel(this);
    QDockWidget* info_dock = new QDockWidget("Info", this);
    info_dock->setWidget(info_panel_);
    this->addDockWidget(Qt::LeftDockWidgetArea, info_dock);

    // Settings and planner panel at the right side
    QDockWidget* tab_dock = new QDockWidget("Settings", this);
    panel_tab_ = new QTabWidget(tab_dock);
    settings_panel_ = new ViewerSettingsPanel(panel_tab_);
    panel_tab_->addTab(settings_panel_, "Settings");
    planner_panel_ = new ViewerPlannerPanel(panel_tab_);
    panel_tab_->addTab(planner_panel_, "Planner");
    tab_dock->setWidget(panel_tab_);
    this->addDockWidget(Qt::RightDockWidgetArea, tab_dock);

    viewer_widget_ = ViewerWidget::create(planner_, settings_panel_, planner_panel_, this);
    this->setCentralWidget(viewer_widget_);
    viewer_widget_->showOctree(planner_->getOctree());
    viewer_widget_->showSparseReconstruction(planner_->getReconstruction());
    if (planner_->getDensePoints() != nullptr) {
      viewer_widget_->showDensePoints(planner_->getDensePoints());
    }
    if (planner_->getMesh() != nullptr) {
      viewer_widget_->showPoissonMesh(planner_->getMesh());
    }

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

const ViewerWidget* ViewerWindow::getViewerWidget() const {
  return viewer_widget_;
}

ViewerWidget* ViewerWindow::getViewerWidget() {
  return viewer_widget_;
}

