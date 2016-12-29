/*
 * viewpoint_drawer.h
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#pragma once

#include "../reconstruction/sparse_reconstruction.h"
#include "triangle_drawer.h"
#include "line_drawer.h"

template <typename FloatType>
class ViewpointDrawer {
  const FloatType CAMERA_SIZE_SPEED = 0.1f;
  const FloatType MIN_CAMERA_SIZE = 0.01f;
  const FloatType MAX_CAMERA_SIZE = 10.0f;
  const FloatType CAMERA_LINE_WIDTH = 1.0f;

  USE_FIXED_EIGEN_TYPES(FloatType)

public:
  ViewpointDrawer();

  ~ViewpointDrawer();

  void setColor(const Vector4& color);

  void setColor(FloatType r, FloatType g, FloatType b, FloatType a);

  void setCamera(const reconstruction::PinholeCamera& camera);

  void setViewpoints(const std::vector<ait::Pose<FloatType>>& poses);

  void changeCameraSize(const FloatType delta);

  void setCameraSize(FloatType camera_size);

  void setDrawCameras(bool draw_cameras);

  void clear();

  void init();

  void upload();

  void draw(const QMatrix4x4& pvm_matrix, const int width, const int height);

private:
  void uploadCameraData();

  void generateImageModel(const reconstruction::PinholeCamera& camera, const ait::Pose<FloatType>& pose,
      const FloatType camera_size, const Vector4& color,
      std::array<OGLLineData, 8>& lines, std::array<OGLTriangleData, 2>& triangles);

  reconstruction::PinholeCamera camera_;
  std::vector<ait::Pose<FloatType>> poses_;
  FloatType camera_size_;
  bool draw_cameras_;
  TriangleDrawer camera_triangle_drawer_;
  LineDrawer camera_line_drawer_;

  Vector4 color_;
};

#include "viewpoint_drawer.hxx"
