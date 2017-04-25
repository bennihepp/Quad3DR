//==================================================
// viewpoint_planner_opengl.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 5, 2017
//==================================================

#include <bh/color.h>
#include "viewpoint_planner.h"

std::unique_lock<std::mutex> ViewpointPlanner::acquireOpenGLLock() const {
  return offscreen_renderer_->acquireOpenGLLock();
}

QImage ViewpointPlanner::drawPoissonMesh(const ViewpointEntryIndex viewpoint_index) const {
  return offscreen_renderer_->drawPoissonMesh(viewpoint_entries_[viewpoint_index].viewpoint);
}

QImage ViewpointPlanner::drawPoissonMeshNormals(const ViewpointEntryIndex viewpoint_index) const {
  return offscreen_renderer_->drawPoissonMeshNormals(viewpoint_entries_[viewpoint_index].viewpoint);
}

QImage ViewpointPlanner::drawPoissonMeshDepth(const ViewpointEntryIndex viewpoint_index) const {
  return offscreen_renderer_->drawPoissonMeshDepth(viewpoint_entries_[viewpoint_index].viewpoint);
}

ViewpointPlanner::Vector3 ViewpointPlanner::computePoissonMeshNormalVector(
    const Viewpoint& viewpoint, const Vector3& position) const {
  return offscreen_renderer_->computePoissonMeshNormalVector(viewpoint, position);
}

ViewpointPlanner::Vector3 ViewpointPlanner::computePoissonMeshNormalVector(
    const Viewpoint& viewpoint,
    const std::size_t x, const std::size_t y) const {
  return offscreen_renderer_->computePoissonMeshNormalVector(viewpoint, x, y);
}

ViewpointPlanner::FloatType ViewpointPlanner::computePoissonMeshDepth(
    const Viewpoint& viewpoint, const Vector3& position) const {
  return offscreen_renderer_->computePoissonMeshDepth(viewpoint, position);
}

ViewpointPlanner::FloatType ViewpointPlanner::computePoissonMeshDepth(
    const Viewpoint& viewpoint,
    const std::size_t x, const std::size_t y) const {
  return offscreen_renderer_->computePoissonMeshDepth(viewpoint, x, y);
}
