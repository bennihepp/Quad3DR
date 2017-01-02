/*
 * viewpoint_drawer.cpp
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#include <array>
#include <iostream>
#include <QtOpenGL>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <ait/common.h>
#include <ait/utilities.h>

template <typename FloatType>
ViewpointDrawer<FloatType>::ViewpointDrawer()
: camera_size_(0.5f), draw_cameras_(true) {}

template <typename FloatType>
ViewpointDrawer<FloatType>::~ViewpointDrawer() {
  clear();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setCamera(const reconstruction::PinholeCamera& camera) {
  camera_ = camera;
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setViewpoints(const std::vector<ait::Pose<FloatType>>& poses) {
  poses_ = poses;
  setColor(1, 0, 0, 0);
  upload();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setViewpoints(const std::vector<ait::Pose<FloatType>>& poses, const Color4& color) {
  poses_ = poses;
  setColor(color);
  upload();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setViewpoints(const std::vector<ait::Pose<FloatType>>& poses, const std::vector<Color4>& colors) {
  poses_ = poses;
  setColors(colors);
  upload();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setColor(FloatType r, FloatType g, FloatType b, FloatType a) {
  Color4 color;
  color << r, g, b, a;
  setColor(color);
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setColor(const Color4& color) {
  colors_.resize(poses_.size(), color);
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setColors(const std::vector<Color4>& colors) {
  AIT_ASSERT(colors.size() == poses_.size());
  colors_ = colors;
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::changeCameraSize(const FloatType delta) {
  if (delta == 0.0f) {
    return;
  }
  camera_size_ *= (1.0f + delta / 100.0f * CAMERA_SIZE_SPEED);
  camera_size_ = ait::clamp(camera_size_, MIN_CAMERA_SIZE, MAX_CAMERA_SIZE);
  uploadCameraData();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setCameraSize(FloatType camera_size) {
  camera_size_ = camera_size;
  uploadCameraData();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::setDrawCameras(bool draw_cameras) {
  draw_cameras_ =  draw_cameras;
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::clear() {
  camera_triangle_drawer_.clear();
  camera_line_drawer_.clear();
  poses_.clear();
  colors_.clear();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::init() {
  camera_triangle_drawer_.init();
  camera_line_drawer_.init();
  upload();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::upload() {
  uploadCameraData();
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::draw(const QMatrix4x4& pvm_matrix, const int width, const int height) {
  if (draw_cameras_) {
    camera_triangle_drawer_.draw(pvm_matrix);
    camera_line_drawer_.draw(pvm_matrix, width, height, CAMERA_LINE_WIDTH);
  }
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::uploadCameraData() {
  if (!camera_.isValid()) {
    return;
  }
//  std::cout << "intrinsics: " << camera_.intrinsics() << std::endl;
  std::vector<OGLTriangleData> triangle_data;
  triangle_data.reserve(2 * poses_.size());
  std::vector<OGLLineData> line_data;
  line_data.reserve(8 * poses_.size());

  for (auto it = poses_.cbegin(); it != poses_.cend(); ++it) {
    const Pose& pose = *it;
    const Color4& color = colors_[it - poses_.cbegin()];
    std::array<OGLLineData, 8> lines;
    std::array<OGLTriangleData, 2> triangles;
    generateImageModel(camera_, pose, camera_size_, color, lines, triangles);

    for (const OGLLineData& line : lines) {
      line_data.push_back(line);
    }
    for (const OGLTriangleData& triangle : triangles) {
      triangle_data.push_back(triangle);
    }
  }

  camera_triangle_drawer_.upload(triangle_data);
  camera_line_drawer_.upload(line_data);
}

template <typename FloatType>
void ViewpointDrawer<FloatType>::generateImageModel(
    const reconstruction::PinholeCamera& camera, const ait::Pose<FloatType>& pose,
    const FloatType camera_size, const Color4& color,
    std::array<OGLLineData, 8>& lines, std::array<OGLTriangleData, 2>& triangles) {
  // Generate camera frustum in OpenGL coordinates
  const FloatType image_width = camera_size * camera.width() / 1024.0f;
  const FloatType image_height =
      image_width * static_cast<FloatType>(camera.height()) / camera.width();
  const FloatType image_extent = std::max(image_width, image_height);
  const FloatType camera_extent = std::max(camera.width(), camera.height());
  const FloatType camera_extent_normalized =
      static_cast<FloatType>(camera_extent / camera.getMeanFocalLength());
  const FloatType focal_length = 2.0f * image_extent / camera_extent_normalized;

  const Matrix3x4 transform_image_to_world =
      pose.getTransformationImageToWorld();
//        std::cout << "transform_world_to_image=" << transform_world_to_image << std::endl;

  // Projection center, top-left, top-right, bottom-right, bottom-left corners
  const Vector3 pc = transform_image_to_world.template rightCols<1>();
  const Vector3 tl = transform_image_to_world
                             * Vector4(-image_width, image_height, focal_length, 1);
  const Vector3 tr = transform_image_to_world
                             * Vector4(image_width, image_height, focal_length, 1);
  const Vector3 br = transform_image_to_world
                             * Vector4(image_width, -image_height, focal_length, 1);
  const Vector3 bl = transform_image_to_world
                             * Vector4(-image_width, -image_height, focal_length, 1);

//        std::cout << "pc=" << pc << std::endl;
//        std::cout << "tl=" << tl << std::endl;
//        std::cout << "tr=" << tr << std::endl;
//        std::cout << "br=" << br << std::endl;
//        std::cout << "bl=" << bl << std::endl;

  const FloatType r = color.r();
  const FloatType g = color.g();
  const FloatType b = color.b();
  const FloatType a = color.a();

  // Lines from sensor corners to projection center
  lines[0].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[0].vertex2 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);

  lines[1].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[1].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);

  lines[2].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[2].vertex2 = OGLVertexDataRGBA(br(0), br(1), br(2), 0.8f * r, g, b, 1);

  lines[3].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[3].vertex2 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);

  lines[4].vertex1 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);
  lines[4].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);

  lines[5].vertex1 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);
  lines[5].vertex2 = OGLVertexDataRGBA(br(0), br(1), br(2), 0.8f * r, g, b, 1);

  lines[6].vertex1 = OGLVertexDataRGBA(br(0), br(1), br(2), 0.8f * r, g, b, 1);
  lines[6].vertex2 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);

  lines[7].vertex1 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);
  lines[7].vertex2 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);

  // Sensor rectangle
  triangles[0].vertex1 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), r, g, b, a);
  triangles[0].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), r, g, b, a);
  triangles[0].vertex3 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), r, g, b, a);

  triangles[1].vertex1 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), r, g, b, a);
  triangles[1].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), r, g, b, a);
  triangles[1].vertex3 = OGLVertexDataRGBA(br(0), br(1), br(2), r, g, b, a);
}
