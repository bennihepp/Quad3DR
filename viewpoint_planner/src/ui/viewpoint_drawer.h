/*
 * viewpoint_drawer.h
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#pragma once

#include "../reconstruction/sparse_reconstruction.h"
#include "../triangle_drawer.h"
#include "../line_drawer.h"

class ViewpointDrawer {
  const float CAMERA_SIZE_SPEED = 0.1f;
  const float MIN_CAMERA_SIZE = 0.01f;
  const float MAX_CAMERA_SIZE = 10.0f;
  const float CAMERA_LINE_WIDTH = 1.0f;

public:
  ViewpointDrawer();

  ~ViewpointDrawer();

  void setCamera(const PinholeCamera& camera);

  void setViewpoints(const std::vector<ait::Pose>& poses);

  void changeCameraSize(const float delta);

  void setCameraSize(float camera_size);

  void setDrawCameras(bool draw_cameras);

  void clear();

  void init();

  void upload();

  void draw(const QMatrix4x4& pvm_matrix, const int width, const int height);

private:
  const float IMAGE_R = 0.0f;
  const float IMAGE_G = 0.1f;
  const float IMAGE_B = 0.8f;
  const float IMAGE_A = 0.6f;

  void uploadCameraData();

  void generateImageModel(const PinholeCamera& camera, const ait::Pose& pose,
      const float camera_size, const float r, const float g, const float b, const float a,
      std::array<OGLLineData, 8>& lines, std::array<OGLTriangleData, 2>& triangles);

  PinholeCamera camera_;
  std::vector<ait::Pose> poses_;
  float camera_size_;
  bool draw_cameras_;
  TriangleDrawer camera_triangle_drawer_;
  LineDrawer camera_line_drawer_;
};
