//==================================================
// offscreen_octree_renderer.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 19.04.17
//==================================================
#pragma once

#include "viewpoint_planner_types.h"

#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>
#include <bh/qt/utils.h>
#include <bh/color.h>
#include <bh/config_options.h>
#include "../rendering/voxel_drawer.h"

namespace viewpoint_planner {

class OffscreenOctreeRenderer {
public:
  using OctreeType = viewpoint_planner::OccupancyMapType;

  struct Options : bh::ConfigOptions {
    Options() {
      addOption<bool>("dump_voxel_image", &dump_voxel_image);
    }

    ~Options() override {}

    // Whether to dump the voxel images after rendering
    bool dump_voxel_image = false;
  };

  explicit OffscreenOctreeRenderer(const OctreeType* octree);

  explicit OffscreenOctreeRenderer(const Options& options, const OctreeType* octree);

  virtual ~OffscreenOctreeRenderer();

  void initialize();

  template <typename OpenGLFunctions>
  void drawOctree(const QMatrix4x4& pvm_matrix, OpenGLFunctions&& opengl_functions) const;

  const rendering::VoxelDrawer& getVoxelDrawer() const { return voxel_drawer_; }

  rendering::VoxelDrawer& getVoxelDrawer() { return voxel_drawer_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Options options_;

  const OctreeType* octree_;
  rendering::VoxelDrawer voxel_drawer_;
};

}