//==================================================
// viewpoint_planner_window.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <octomap/octomap.h>
#include <qglviewer.h>
#include <QMainWindow>
#include "viewer_widget.h"
#include "viewer_info_panel.h"
#include "viewer_settings_panel.h"
#include "viewpoint_planner.h"

class ViewerWindow : public QMainWindow {
  Q_OBJECT

 public:

  ViewerWindow(ViewpointPlanner* planner, QWidget *parent = nullptr);
  ~ViewerWindow();

protected slots:
  void onCameraPoseTimeout();
  void onCameraPoseTimeoutHandlerFinished();

signals:
  void cameraPoseTimeoutHandlerFinished();

protected:
   ViewpointPlanner* planner_;
   ViewerInfoPanel* info_panel_;
   ViewerSettingsPanel* settings_panel_;
   ViewerWidget* viewer_widget_;
   QTimer* camera_pose_timer_;
   std::thread worker_thread_;
};
