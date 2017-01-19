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
#include <ait/options.h>

#include "../planner/viewpoint_planner.h"
#include "viewer_widget.h"
#include "viewer_info_panel.h"
#include "viewer_settings_panel.h"
#include "viewer_planner_panel.h"

class ViewerWindow : public QMainWindow {
  Q_OBJECT

 public:

  using Options = ViewerWidget::Options;

  ViewerWindow(const Options& options, ViewpointPlanner* planner, QWidget *parent = nullptr);

  ~ViewerWindow();

  const ViewerWidget* getViewerWidget() const;

  ViewerWidget* getViewerWidget();

protected:
   ViewpointPlanner* planner_;
   ViewerWidget* viewer_widget_;
   QTabWidget* panel_tab_;
   ViewerInfoPanel* info_panel_;
   ViewerSettingsPanel* settings_panel_;
   ViewerPlannerPanel* planner_panel_;
};
