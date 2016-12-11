//==================================================
// viewpoint_planner_window.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================

#include <QDockWidget>
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

    viewer_widget_ = ViewerWidget::create(settings_panel_, this);
    this->setCentralWidget(viewer_widget_);
    viewer_widget_->showOctree(planner_->getOctree());
    viewer_widget_->showSparseReconstruction(planner_->getSparseReconstruction());

    // Update labels in info panel
    info_panel_->setVoxelSize(planner_->getOctree()->getResolution());
    info_panel_->setNumOfNodes(planner_->getOctree()->size());
    info_panel_->setNumOfLeafNodes(planner_->getOctree()->getNumLeafNodes());
    info_panel_->setTreeDepth(planner_->getOctree()->getTreeDepth());
    double extent_x, extent_y, extent_z;
    planner_->getOctree()->getMetricSize(extent_x, extent_y, extent_z);
    info_panel_->setExtent(QVector3D(extent_x, extent_y, extent_z));
    info_panel_->setNumOfImages(planner_->getSparseReconstruction()->getImages().size());
    info_panel_->setNumOfSparsePoints(planner_->getSparseReconstruction()->getPoints3D().size());

    // Timer for getting camera pose updates
    camera_pose_timer_ = new QTimer(this);
    camera_pose_timer_->setSingleShot(true);
    connect(camera_pose_timer_, SIGNAL(timeout()), this, SLOT(onCameraPoseTimeout()));
    connect(this, SIGNAL(cameraPoseTimeoutHandlerFinished()), this, SLOT(onCameraPoseTimeoutHandlerFinished()));
    emit cameraPoseTimeoutHandlerFinished();
}

ViewerWindow::~ViewerWindow() {
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void ViewerWindow::onCameraPoseTimeoutHandlerFinished() {
  camera_pose_timer_->start(500);
}

void ViewerWindow::onCameraPoseTimeout() {
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  worker_thread_ = std::thread([this]() {
    Pose camera_pose = viewer_widget_->getCameraPose();
    std::cout << "pose_matrix image to world: " << camera_pose.getTransformationImageToWorld() << std::endl;
    std::cout << "translation: " << camera_pose.inverse().translation();
    CameraId camera_id = planner_->getSparseReconstruction()->getCameras().cbegin()->first;
    std::unordered_set<Point3DId> proj_points = planner_->computeProjectedMapPoints(camera_id, Pose(camera_pose));
    std::unordered_set<Point3DId> filtered_points = planner_->computeFilteredMapPoints(camera_id, Pose(camera_pose));
    std::unordered_set<Point3DId> visible_points = planner_->computeVisibleMapPoints(camera_id, Pose(camera_pose));

    std::cout << "  projected points: " << proj_points.size() << std::endl;
    std::cout << "  filtered points: " << filtered_points.size() << std::endl;
    std::cout << "  non-occluded points: " << visible_points.size() << std::endl;
    std::cout << "  filtered and non-occluded: " << planner_->setIntersectionSize(filtered_points, visible_points) << std::endl;

    emit this->cameraPoseTimeoutHandlerFinished();
  });
}
