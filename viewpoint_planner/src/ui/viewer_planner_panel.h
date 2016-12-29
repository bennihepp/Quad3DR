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
        connect(ui.pauseContinueViewpointGraph, SIGNAL(clicked(void)), this, SLOT(pauseContinueViewpointGraphInternal()));
        connect(ui.pauseContinueViewpointPath, SIGNAL(clicked(void)), this, SLOT(pauseContinueViewpointPathInternal()));
        connect(ui.resetViewpoints, SIGNAL(clicked(void)), this, SLOT(resetViewpointsInternal()));
        connect(ui.resetViewpointPath, SIGNAL(clicked(void)), this, SLOT(resetViewpointPathInternal()));
        connect(ui.drawViewpointGraph, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointGraphInternal(int)));
        connect(ui.viewpointGraphSelection, SIGNAL(activated(int)), this, SLOT(setViewpointGraphSelectionInternal(int)));
        connect(ui.drawViewpointPath, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointPathInternal(int)));
        connect(ui.viewpointPathSelection, SIGNAL(activated(int)), this, SLOT(setViewpointPathSelectionInternal(int)));
    }

    ~ViewerPlannerPanel() {
    }

    void setPauseContinueViewpointGraphEnabled(bool enabled) {
      ui.pauseContinueViewpointGraph->setEnabled(enabled);
    }

    void setPauseContinueViewpointGraphText(const std::string& text) {
      ui.pauseContinueViewpointGraph->setText(QString::fromStdString(text));
    }

    void setPauseContinueViewpointPathEnabled(bool enabled) {
      ui.pauseContinueViewpointPath->setEnabled(enabled);
    }

    void setPauseContinueViewpointPathText(const std::string& text) {
      ui.pauseContinueViewpointPath->setText(QString::fromStdString(text));
    }

    void setResetViewpointsEnabled(bool enabled) {
      ui.resetViewpoints->setEnabled(enabled);
    }

    void setResetViewpointPathEnabled(bool enabled) {
      ui.resetViewpointPath->setEnabled(enabled);
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

    void setViewpointGraphSelectionInternal(int index) {
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

    void setViewpointPathSelectionInternal(int index) {
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

  void pauseContinueViewpointGraphInternal() {
    emit pauseContinueViewpointGraph();
  }

  void pauseContinueViewpointPathInternal() {
    emit pauseContinueViewpointPath();
  }

  void resetViewpointsInternal() {
    emit resetViewpoints();
  }

  void resetViewpointPathInternal() {
    emit resetViewpointPath();
  }

signals:
  void pauseContinueViewpointGraph();
  void pauseContinueViewpointPath();
  void drawViewpointGraphChanged(bool draw_viewpoint_graph_changed);
  void viewpointGraphSelectionChanged(size_t index);
  void drawViewpointPathChanged(bool draw_viewpoint_path_changed);
  void viewpointPathSelectionChanged(size_t index);
  void resetViewpoints();
  void resetViewpointPath();

private:
    Ui::ViewerPlannerPanelClass ui;
};
