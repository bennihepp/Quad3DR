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
#include <QDateTime>
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
      connect(ui.solveViewpointTSP, SIGNAL(clicked(void)), this, SLOT(solveViewpointTSPInternal()));
      connect(ui.resetViewpoints, SIGNAL(clicked(void)), this, SLOT(resetViewpointsInternal()));
      connect(ui.resetViewpointMotions, SIGNAL(clicked(void)), this, SLOT(resetViewpointMotionsInternal()));
      connect(ui.resetViewpointPath, SIGNAL(clicked(void)), this, SLOT(resetViewpointPathInternal()));
      connect(ui.saveViewpointGraph, SIGNAL(clicked(void)), this, SLOT(saveViewpointGraphInternal()));
      connect(ui.loadViewpointGraph, SIGNAL(clicked(void)), this, SLOT(loadViewpointGraphInternal()));
      connect(ui.saveViewpointPath, SIGNAL(clicked(void)), this, SLOT(saveViewpointPathInternal()));
      connect(ui.loadViewpointPath, SIGNAL(clicked(void)), this, SLOT(loadViewpointPathInternal()));
      connect(ui.exportBestViewpointPathAsJson, SIGNAL(clicked(void)), this, SLOT(exportViewpointPathAsJsonInternal()));
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
      connect(ui.viewpointColorMode, SIGNAL(currentIndexChanged(int)), this, SLOT(setViewpointColorModeInternal(int)));
      connect(ui.viewpointGraphComponent, SIGNAL(currentIndexChanged(int)), this, SLOT(setViewpointGraphComponentInternal(int)));
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

  void setSolveViewpointTSPEnabled(bool enabled) {
    ui.solveViewpointTSP->setEnabled(enabled);
  }

  void setSolveViewpointTSPText(const std::string& text) {
    ui.solveViewpointTSP->setText(QString::fromStdString(text));
  }

  void setAllComputationButtonsEnabled(bool enabled) {
    setPauseContinueViewpointGraphEnabled(enabled);
    setPauseContinueViewpointMotionsEnabled(enabled);
    setPauseContinueViewpointPathEnabled(enabled);
    setSolveViewpointTSPEnabled(enabled);
  }

  void setAllComputationButtonsText(const std::string& text) {
    setPauseContinueViewpointGraphText(text);
    setPauseContinueViewpointMotionsText(text);
    setPauseContinueViewpointPathText(text);
    setSolveViewpointTSPText(text);
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

  void setAllResetButtonsEnabled(bool enabled) {
    setResetViewpointsEnabled(enabled);
    setResetViewpointMotionsEnabled(enabled);
    setResetViewpointPathEnabled(enabled);
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

  std::size_t getViewpointGraphSelection() {
    return static_cast<std::size_t>(ui.viewpointGraphSelection->currentIndex());
  }

  void setViewpointGraphSelection(const std::size_t index) {
    selectItemByUserData(ui.viewpointGraphSelection, index);
  }

  std::size_t getViewpointPathBranchSelection() {
    return static_cast<std::size_t>(ui.viewpointPathBranchSelection->currentIndex());
  }

  void setViewpointPathBranchSelection(const std::size_t index) {
    selectItemByUserData(ui.viewpointPathBranchSelection, index);
  }

  std::size_t getViewpointPathSelection() {
    return static_cast<std::size_t>(ui.viewpointPathSelection->currentIndex());
  }

  void setViewpointPathSelectionByItemIndex(int index) {
    ui.viewpointPathSelection->setCurrentIndex(index);
  }

  void setViewpointPathSelection(const std::size_t index) {
    selectItemByUserData(ui.viewpointPathSelection, index);
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

  void setViewpointMotionsSize(std::size_t viewpoint_motions_size) {
    ui.viewpointMotionsSize->setText(QString::number(viewpoint_motions_size));
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

  bool isShowIncrementalVoxelSetChecked() const {
    return ui.showIncrementalVoxelSet->isChecked();
  }

  bool isShowAccumulativeVoxelSetChecked() const {
    return ui.showAccumulatedVoxelSet->isChecked();
  }

  bool isInspectViewpointGraphMotionsChecked() const {
    return ui.inspectViewpointGraphMotions->isChecked();
  }

  bool isUpdateCameraOnSelectionChecked() const {
    return ui.updateCameraOnSelection->isChecked();
  }

  double getAlphaParameter() const {
    return ui.alphaParameter->value();
  }

  double getBetaParameter() const {
    return ui.betaParameter->value();
  }

  void setAlphaParameter(double alpha_parameter) const {
    ui.alphaParameter->setValue(alpha_parameter);
  }

  void setBetaParameter(double beta_parameter) const {
    ui.betaParameter->setValue(beta_parameter);
  }

  double getMinInformationFilter() const {
    return ui.minInformationFilter->value();
  }

  double getViewpointPathLineWidth() const {
    return ui.viewpointPathLineWidth->value();
  }

  void selectViewpointColorMode(const std::size_t color_mode) {
    for (std::size_t i = 0; i < static_cast<std::size_t>(ui.viewpointColorMode->count()); ++i) {
      std::size_t entry_mode = ui.viewpointColorMode->itemData(i).toUInt();
      if (color_mode == entry_mode) {
        ui.viewpointColorMode->setCurrentIndex(i);
        return;
      }
    }
    throw AIT_EXCEPTION(std::string("Could not find corresponding color mode in combo box: ") + std::to_string(color_mode));
  }

  void initializeViewpointColorMode(std::vector<std::pair<std::string, std::size_t>>& entries) {
    ui.viewpointColorMode->blockSignals(true);
    ui.viewpointColorMode->clear();
    for (const auto& entry : entries) {
        ui.viewpointColorMode->addItem(QString::fromStdString(entry.first), QVariant(static_cast<uint32_t>(entry.second)));
    }
    ui.viewpointColorMode->blockSignals(false);
  }

  int getViewpointComponentSelection() {
    return static_cast<int>(ui.viewpointGraphComponent->currentIndex());
  }

  void setViewpointComponentSelectionByItemIndex(int index) {
    ui.viewpointGraphComponent->setCurrentIndex(index);
  }

  void setViewpointComponentSelection(int index) {
    index = std::min(index, ui.viewpointGraphComponent->count());
    selectItemByUserData(ui.viewpointGraphComponent, index);
  }

  void initializeViewpointComponents(std::vector<std::pair<std::string, int>>& entries) {
    ui.viewpointGraphComponent->blockSignals(true);
    ui.viewpointGraphComponent->clear();
    for (const auto& entry : entries) {
        ui.viewpointGraphComponent->addItem(QString::fromStdString(entry.first), QVariant(entry.second));
    }
    ui.viewpointGraphComponent->blockSignals(false);
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

  void solveViewpointTSPInternal() {
    emit solveViewpointTSP();
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
    QString filename = QFileDialog::getOpenFileName(this, tr("Load viewpoint graph"),
        QString(), tr("Boost Serialization File (*.bs);;All Files (*.*)"));
    if (!filename.isNull()) {
      emit loadViewpointGraph(filename.toStdString());
    }
  }

  void saveViewpointPathInternal() {
    const std::string format_ext = ".bs";
    const std::string date_string = QDateTime::currentDateTime().toString("dd.MM.yy_hh.mm.ss").toStdString();
    std::string suggested_filename = std::string("viewpoint_path_") + date_string + format_ext;
    QString filename = QFileDialog::getSaveFileName(this, tr("Save viewpoint path"),
        QString::fromStdString(suggested_filename), tr("Boost Serialization File (*.bs);;All Files (*.*)"));
    if (!filename.isNull()) {
      emit saveViewpointPath(filename.toStdString());
    }
  }

  void loadViewpointPathInternal() {
    QString filename = QFileDialog::getOpenFileName(this, tr("Load viewpoint path"),
        QString(), tr("Boost Serialization File (*.bs);;All Files (*.*)"));
    if (!filename.isNull()) {
      emit loadViewpointPath(filename.toStdString());
    }
  }

  void exportViewpointPathAsJsonInternal() {
    const std::string format_ext = ".json";
    const std::string date_string = QDateTime::currentDateTime().toString("dd.MM.yy_hh.mm.ss").toStdString();
    std::string suggested_filename = std::string("viewpoint_path_") + date_string + format_ext;
    QString filename = QFileDialog::getSaveFileName(this, tr("Export viewpoint path as JSON"),
        QString::fromStdString(suggested_filename), tr("JSON File (*.json);;All Files (*.*)"));
    if (!filename.isNull()) {
      emit exportViewpointPathAsJson(filename.toStdString());
    }
  }

  void setViewpointColorModeInternal(int index) {
      bool ok;
      std::size_t color_mode = ui.viewpointColorMode->itemData(index).toUInt(&ok);
      if (!ok) {
          throw AIT_EXCEPTION("Unable to convert color mode user data to uint");
      }
      emit viewpointColorModeChanged(color_mode);
  }

  void setViewpointGraphComponentInternal(int index) {
      bool ok;
      int component = ui.viewpointGraphComponent->itemData(index).toInt(&ok);
      if (!ok) {
          throw AIT_EXCEPTION("Unable to convert graph component user data to int");
      }
      emit viewpointGraphComponentChanged(component);
  }

signals:
  void pauseContinueViewpointGraph();
  void pauseContinueViewpointMotions();
  void pauseContinueViewpointPath();
  void solveViewpointTSP();
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
  void saveViewpointPath(const std::string& filename);
  void loadViewpointPath(const std::string& filename);
  void exportViewpointPathAsJson(const std::string& filename);
  void alphaParameterChanged(double alpha);
  void betaParameterChanged(double beta);
  void minInformationFilterChanged(double min_information_filter);
  void viewpointPathLineWidthChanged(double line_width);
  void viewpointColorModeChanged(std::size_t color_mode);
  void viewpointGraphComponentChanged(int component);

private:
  template <typename T>
  void selectItemByUserData(QComboBox* combo_box, const T item_user_data) {
    combo_box->blockSignals(true);
    for (int i = 0; i < combo_box->count(); ++i) {
      bool ok;
      int user_data = combo_box->itemData(i).toInt(&ok);
      if (!ok) {
        throw AIT_EXCEPTION("Unable to convert combo box user data user data to int");
      }
      if (static_cast<T>(user_data) == item_user_data) {
        combo_box->setCurrentIndex(i);
        break;
      }
    }
    combo_box->blockSignals(false);
  }

    Ui::ViewerPlannerPanelClass ui;
};
