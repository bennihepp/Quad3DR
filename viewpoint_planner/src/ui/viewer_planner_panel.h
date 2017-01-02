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
#include <QFileDialog>
#include <ait/common.h>
#include "ui_viewer_planner_panel.h"

class ViewerPlannerPanel : public QWidget {
  Q_OBJECT

public:
  ViewerPlannerPanel(QWidget *parent = 0)
  : QWidget(parent) {
      ui.setupUi(this);
      connect(ui.pauseContinueViewpointGraph, SIGNAL(clicked(void)), this, SLOT(pauseContinueViewpointGraphInternal()));
      connect(ui.pauseContinueViewpointMotions, SIGNAL(clicked(void)), this, SLOT(pauseContinueViewpointMotionsInternal()));
      connect(ui.pauseContinueViewpointPath, SIGNAL(clicked(void)), this, SLOT(pauseContinueViewpointPathInternal()));
      connect(ui.resetViewpoints, SIGNAL(clicked(void)), this, SLOT(resetViewpointsInternal()));
      connect(ui.resetViewpointPath, SIGNAL(clicked(void)), this, SLOT(resetViewpointPathInternal()));
      connect(ui.saveViewpointGraph, SIGNAL(clicked(void)), this, SLOT(saveViewpointGraphInternal()));
      connect(ui.loadViewpointGraph, SIGNAL(clicked(void)), this, SLOT(loadViewpointGraphInternal()));
      connect(ui.drawViewpointGraph, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointGraphInternal(int)));
      connect(ui.viewpointGraphSelection, SIGNAL(activated(int)), this, SLOT(setViewpointGraphSelectionInternal(int)));
      connect(ui.drawViewpointMotions, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointMotionsInternal(int)));
      connect(ui.viewpointPathBranchSelection, SIGNAL(activated(int)), this, SLOT(setViewpointPathBranchInternal(int)));
      connect(ui.drawViewpointPath, SIGNAL(stateChanged(int)), this, SLOT(setDrawViewpointPathInternal(int)));
      connect(ui.viewpointPathSelection, SIGNAL(activated(int)), this, SLOT(setViewpointPathSelectionInternal(int)));
      connect(ui.useFixedColors, SIGNAL(stateChanged(int)), this, SLOT(setUseFixedColorsInternal(int)));
      connect(ui.alphaParameter, SIGNAL(valueChanged(double)), this, SIGNAL(alphaParameterChanged(double)));
      connect(ui.betaParameter, SIGNAL(valueChanged(double)), this, SIGNAL(betaParameterChanged(double)));
      connect(ui.minInformationFilter, SIGNAL(valueChanged(double)), this, SIGNAL(minInformationFilterChanged(double)));
      connect(ui.viewpointPathLineWidth, SIGNAL(valueChanged(double)), this, SIGNAL(viewpointPathLineWidthChanged(double)));
  }

  ~ViewerPlannerPanel() {
  }

  void setPauseContinueViewpointGraphEnabled(bool enabled) {
    ui.pauseContinueViewpointGraph->setEnabled(enabled);
  }

  void setPauseContinueViewpointMotionsEnabled(bool enabled) {
    ui.pauseContinueViewpointMotions->setEnabled(enabled);
  }

  void setPauseContinueViewpointGraphText(const std::string& text) {
    ui.pauseContinueViewpointGraph->setText(QString::fromStdString(text));
  }

  void setPauseContinueViewpointMotionsText(const std::string& text) {
    ui.pauseContinueViewpointMotions->setText(QString::fromStdString(text));
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

  void setResetViewpointMotionsEnabled(bool enabled) {
    ui.resetViewpointMotions->setEnabled(enabled);
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

  void initializeViewpointPathBranch(const std::vector<std::pair<std::string, size_t>>& entries) {
    ui.viewpointPathBranchSelection->blockSignals(true);
    ui.viewpointPathBranchSelection->clear();
    for (const auto& entry : entries) {
      ui.viewpointPathBranchSelection->addItem(QString::fromStdString(entry.first), QVariant(static_cast<int>(entry.second)));
    }
    ui.viewpointPathBranchSelection->blockSignals(false);
  }

  void setViewpointPathBranchSelection(std::size_t index) {
    ui.viewpointPathBranchSelection->blockSignals(true);
    ui.viewpointPathBranchSelection->setCurrentIndex(static_cast<int>(index));
    ui.viewpointPathBranchSelection->blockSignals(false);
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

  bool isDrawViewpointGraphChecked() const {
    return ui.drawViewpointGraph->isChecked();
  }

  bool isDrawViewpointMotionsChecked() const {
    return ui.drawViewpointMotions->isChecked();
  }

  bool isDrawViewpointPathChecked() const {
    return ui.drawViewpointPath->isChecked();
  }

  bool isUseFixedColorsChecked() const {
    return ui.useFixedColors->isChecked();
  }

  double getAlphaParameter() const {
    return ui.alphaParameter->value();
  }

  double getBetaParameter() const {
    return ui.betaParameter->value();
  }

  double getMinInformationFilter() const {
    return ui.minInformationFilter->value();
  }

  double getViewpointPathLineWidth() const {
    return ui.viewpointPathLineWidth->value();
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
      std::size_t pose_index = static_cast<std::size_t>(user_data);
      emit viewpointGraphSelectionChanged(pose_index);
    }
  }

  void setDrawViewpointMotionsInternal(int state) {
    emit drawViewpointMotionsChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
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
      std::size_t pose_index = static_cast<std::size_t>(user_data);
      emit viewpointPathSelectionChanged(pose_index);
    }
  }

  void setViewpointPathBranchInternal(int index) {
    bool ok;
    int user_data = ui.viewpointPathBranchSelection->itemData(index).toInt(&ok);
    if (!ok) {
      throw AIT_EXCEPTION("Unable to convert viewpoint path branch to int");
    }
    std::size_t branch_index = static_cast<std::size_t>(user_data);
    emit viewpointPathBranchSelectionChanged(branch_index);
  }

  void setUseFixedColorsInternal(int state) {
    emit useFixedColorsChanged(state == Qt::PartiallyChecked || state == Qt::Checked);
  }

  void pauseContinueViewpointGraphInternal() {
    emit pauseContinueViewpointGraph();
  }

  void pauseContinueViewpointMotionsInternal() {
    emit pauseContinueViewpointMotions();
  }

  void pauseContinueViewpointPathInternal() {
    emit pauseContinueViewpointPath();
  }

  void resetViewpointsInternal() {
    emit resetViewpoints();
  }

  void resetViewpointMotionsInternal() {
    emit resetViewpointMotions();
  }

  void resetViewpointPathInternal() {
    emit resetViewpointPath();
  }

  void saveViewpointGraphInternal() {
    QString filename = QFileDialog::getSaveFileName(this, tr("Save viewpoint graph"),
        "viewpoint_graph.bs", tr("Boost Serialization File (*.bs);;All Files (*.*)"));
    if (!filename.isNull()) {
      emit saveViewpointGraph(filename.toStdString());
    }
  }

  void loadViewpointGraphInternal() {
    QString filename = QFileDialog::getOpenFileName(this, tr("Save viewpoint graph"),
        QString(), tr("Boost Serialization File (*.bs);;All Files (*.*)"));
    if (!filename.isNull()) {
      emit loadViewpointGraph(filename.toStdString());
    }
  }

signals:
  void pauseContinueViewpointGraph();
  void pauseContinueViewpointMotions();
  void pauseContinueViewpointPath();
  void drawViewpointGraphChanged(bool draw_viewpoint_graph_changed);
  void viewpointGraphSelectionChanged(std::size_t index);
  void drawViewpointPathChanged(bool draw_viewpoint_path_changed);
  void viewpointPathBranchSelectionChanged(std::size_t index);
  void viewpointPathSelectionChanged(std::size_t index);
  void drawViewpointMotionsChanged(bool draw_viewpoint_motions_changed);
  void useFixedColorsChanged(bool use_fixed_color);
  void resetViewpoints();
  void resetViewpointMotions();
  void resetViewpointPath();
  void saveViewpointGraph(const std::string& filename);
  void loadViewpointGraph(const std::string& filename);
  void alphaParameterChanged(double alpha);
  void betaParameterChanged(double beta);
  void minInformationFilterChanged(double min_information_filter);
  void viewpointPathLineWidthChanged(double line_width);

private:
    Ui::ViewerPlannerPanelClass ui;
};
