//==================================================
// viewer_planner_panel.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 28, 2016
//==================================================
#pragma once

#include <cmath>
#include <QWidget>
#include <ait/common.h>
#include "ui_viewer_planner_panel.h"

class ViewerPlannerPanel : public QWidget
{
    Q_OBJECT

public:
    ViewerPlannerPanel(QWidget *parent = 0)
    : QWidget(parent) {
        ui.setupUi(this);
        connect(ui.pauseContinuePlanning, SIGNAL(clicked(void)), this, SLOT(pauseContinuePlanningInternal()));
        connect(ui.updateViewpoints, SIGNAL(clicked(void)), this, SIGNAL(updateViewpoints()));
        connect(ui.drawViewpointGraph, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointGraphInternal(int)));
        connect(ui.viewpointGraphSelection, SIGNAL(activated(int)), this, SLOT(setViewpoingGraphSelectionInternal(int)));
        connect(ui.drawViewpointPath, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointPathInternal(int)));
        connect(ui.viewpointPathSelection, SIGNAL(activated(int)), this, SLOT(setViewpoingPathSelectionInternal(int)));
    }

    ~ViewerPlannerPanel() {
    }

    void setPauseContinueText(const std::string& text) {
      ui.pauseContinuePlanning->setText(QString::fromStdString(text));
    }

    void initializeViewpointGraph(const std::vector<std::pair<std::string, size_t>>& entries) {
      ui.viewpointGraphSelection->blockSignals(true);
      ui.viewpointGraphSelection->clear();
      ui.viewpointGraphSelection->addItem(QString::fromStdString("<free view>"), QVariant(static_cast<int>(-1)));
      for (const auto& entry : entries) {
        ui.viewpointGraphSelection->addItem(QString::fromStdString(entry.first), QVariant(static_cast<int>(entry.second)));
      }
      ui.viewpointGraphSelection->blockSignals(false);
    }

    void setViewpointGraphSize(std::size_t viewpoint_graph_size) {
      ui.viewpointGraphSize->setText(QString::number(viewpoint_graph_size));
    }

    void initializeViewpointPath(const std::vector<std::pair<std::string, size_t>>& entries) {
      ui.viewpointPathSelection->blockSignals(true);
      ui.viewpointPathSelection->clear();
      ui.viewpointPathSelection->addItem(QString::fromStdString("<free view>"), QVariant(static_cast<int>(-1)));
      for (const auto& entry : entries) {
        ui.viewpointPathSelection->addItem(QString::fromStdString(entry.first), QVariant(static_cast<int>(entry.second)));
      }
      ui.viewpointPathSelection->blockSignals(false);
    }

    void setViewpointPathSize(std::size_t viewpoint_path_size) {
      ui.viewpointPathSize->setText(QString::number(viewpoint_path_size));
    }

protected slots:
    void setDrawViewpointGraphInternal(int state) {
      emit drawViewpointGraphChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setViewpoingGraphSelectionInternal(int index) {
        bool ok;
        int user_data = ui.viewpointGraphSelection->itemData(index).toInt(&ok);
        if (!ok) {
            throw AIT_EXCEPTION("Unable to convert planned view pose user data user data to int");
        }
        if (user_data >= 0) {
          size_t pose_index = static_cast<size_t>(user_data);
          emit viewpointGraphSelectionChanged(pose_index);
        }
    }

    void setDrawViewpointPathInternal(int state) {
      emit drawViewpointPathChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
    }

    void setViewpoingPathSelectionInternal(int index) {
        bool ok;
        int user_data = ui.viewpointPathSelection->itemData(index).toInt(&ok);
        if (!ok) {
            throw AIT_EXCEPTION("Unable to convert planned view pose user data user data to int");
        }
        if (user_data >= 0) {
          size_t pose_index = static_cast<size_t>(user_data);
          emit viewpointPathSelectionChanged(pose_index);
        }
    }

  void pauseContinuePlanningInternal() {
    emit pauseContinuePlanning();
  }

signals:
  void pauseContinuePlanning();
  void updateViewpoints();
  void drawViewpointGraphChanged(bool draw_viewpoint_graph_changed);
  void viewpointGraphSelectionChanged(size_t index);
  void drawViewpointPathChanged(bool draw_viewpoint_path_changed);
  void viewpointPathSelectionChanged(size_t index);

private:
    Ui::ViewerPlannerPanelClass ui;
};
