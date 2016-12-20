//==================================================
// camera_drawer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 8, 2016
//==================================================
#pragma once

#include "sparse_reconstruction.h"
#include "triangle_drawer.h"
#include "line_drawer.h"

class SparseReconstructionDrawer {
  const float CAMERA_SIZE_SPEED = 0.1f;
  const float MIN_CAMERA_SIZE = 0.01f;
  const float MAX_CAMERA_SIZE = 10.0f;
  const float POINT_SIZE_SPEED = 0.1f;
  const float MIN_POINT_SIZE = 0.1f;
  const float MAX_POINT_SIZE = 100.0f;
  const float CAMERA_LINE_WIDTH = 1.0f;
  const double RENDER_MAX_POINT_ERROR = 2.0;
  const size_t RENDER_MIN_TRACK_LENGTH = 3;

public:
  SparseReconstructionDrawer();

  ~SparseReconstructionDrawer();

  void setSparseReconstruction(const SparseReconstruction* sparse_recon);

  void changeCameraSize(const float delta);

  void changePointSize(const float delta);

  void setCameraSize(float camera_size);

  void setDrawCameras(bool draw_cameras);

  void setDrawSparsePoints(bool draw_sparse_points);

  void clear();

  void init();

  void upload();

  void draw(const QMatrix4x4& pvm_matrix, const int width, const int height);

private:
  const float IMAGE_R = 1.0f;
  const float IMAGE_G = 0.1f;
  const float IMAGE_B = 0.0f;
  const float IMAGE_A = 0.6f;

  void uploadCameraData();

  void uploadPointData();

  void generateImageModel(const PinholeCameraColmap& camera, const Image& image,
      const float camera_size, const float r, const float g, const float b, const float a,
      std::array<OGLLineData, 8>& lines, std::array<OGLTriangleData, 2>& triangles);

  const SparseReconstruction* sparse_recon_;
  float camera_size_;
  float point_size_;
  bool draw_cameras_;
  bool draw_sparse_points_;
  TriangleDrawer camera_triangle_drawer_;
  LineDrawer camera_line_drawer_;
  PointDrawer sparse_point_drawer_;
};
